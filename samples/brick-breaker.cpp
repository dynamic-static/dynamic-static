
/*******************************************************************************

MIT License

Copyright (c) dynamic-static

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*******************************************************************************/

#include "dynamic-static.sample-utilities.hpp"
#include "dynamic-static.physics/context.hpp"
#include "dynamic-static.physics/collider.hpp"
#include "dynamic-static.physics/rigid-body.hpp"
#include "dynamic-static.graphics/primitives.hpp"
#include "dynamic-static/finite-state-machine.hpp"

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

std::string btvecstr(const btVector3& v)
{
    return "{" + std::to_string(v.x()) + ", " + std::to_string(v.y()) + ", " + std::to_string(v.z()) + "}";
}

VkResult create_sphere_mesh(const gvk::Context& context, float radius, uint32_t subdivisions, gvk::Mesh* pMesh)
{
    std::vector<glm::vec3> vertices(dst::gfx::primitive::Icosahedron::Vertices.begin(), dst::gfx::primitive::Icosahedron::Vertices.end());
    for (auto& vertex : vertices) {
        vertex *= radius;
    }
    std::vector<dst::gfx::primitive::Triangle<uint32_t>> triangles(dst::gfx::primitive::Icosahedron::Triangles.begin(), dst::gfx::primitive::Icosahedron::Triangles.end());
    std::unordered_map<dst::gfx::primitive::Edge<uint32_t>, uint32_t, dst::gfx::primitive::EdgeHasher<uint32_t>> edges;
    for (uint32_t subdivision_i = 0; subdivision_i < subdivisions; ++subdivision_i) {
        auto triangleCount = (uint32_t)triangles.size();
        for (uint32_t triangle_i = 0; triangle_i < triangleCount; ++triangle_i) {
            dst::gfx::primitive::subdivide_triangle(
                triangles[triangle_i],
                [&](const dst::gfx::primitive::Edge<uint32_t>& edge)
                {
                    auto itr = edges.find(edge);
                    if (itr == edges.end()) {
                        vertices.push_back(glm::normalize(vertices[edge[0]] + vertices[edge[1]]) * radius);
                        itr = edges.insert(itr, { edge, (uint32_t)vertices.size() - 1 });
                    }
                    return itr->second;
                },
                [&](
                    const dst::gfx::primitive::Triangle<uint32_t>& subdividedTriangle,
                    const std::array<dst::gfx::primitive::Triangle<uint32_t>, 3>& newTriangles
                )
                {
                    triangles[triangle_i] = subdividedTriangle;
                    triangles.insert(triangles.end(), newTriangles.begin(), newTriangles.end());
                }
            );
        }
    }
    return pMesh->write(
        context.get_devices()[0],
        gvk::get_queue_family(context.get_devices()[0], 0).queues[0],
        context.get_command_buffers()[0],
        VK_NULL_HANDLE,
        (uint32_t)vertices.size(),
        vertices.data(),
        (uint32_t)triangles.size() * 3,
        triangles[0].data()
    );
}

VkResult create_box_mesh(const gvk::Context& context, const glm::vec3& dimensions, gvk::Mesh* pMesh)
{
    std::vector<glm::vec3> vertices(dst::gfx::primitive::Cube::Vertices.begin(), dst::gfx::primitive::Cube::Vertices.end());
    for (auto& vertex : vertices) {
        vertex *= dimensions;
    }
    return pMesh->write(
        context.get_devices()[0],
        gvk::get_queue_family(context.get_devices()[0], 0).queues[0],
        context.get_command_buffers()[0],
        VK_NULL_HANDLE,
        (uint32_t)vertices.size(),
        vertices.data(),
        (uint32_t)dst::gfx::primitive::Cube::Triangles.size() * 3,
        dst::gfx::primitive::Cube::Triangles[0].data()
    );
}

struct ObjectUniforms
{
    glm::mat4 world { };
    glm::vec3 color { };
};

struct CameraUniforms
{
    glm::mat4 view { };
    glm::mat4 projection { };
};

class Renderer final
{
public:

};

const int BrickGroup = 1;
const int BallGroup = 2;
const int PaddleGroup = 4;
const int WallGroup = 8;
const int AllGroup = BrickGroup | BallGroup | PaddleGroup | WallGroup;

bool useMotionState = false;

class Object final
{
public:
    void setup_graphics_resources(const gvk::Context& context, const gvk::Mesh& mesh, const gvk::DescriptorSet& descriptorSet, const glm::vec4& color)
    {
        mColor = color;
        mMesh = mesh;
        mDescriptorSet = descriptorSet;
        auto vkResult = dst_sample_create_uniform_buffer<ObjectUniforms>(context, &mUniformBuffer);
        assert(vkResult == VK_SUCCESS);
        auto descriptorBufferInfo = gvk::get_default<VkDescriptorBufferInfo>();
        descriptorBufferInfo.buffer = mUniformBuffer;
        auto writeDescriptorSet = gvk::get_default<VkWriteDescriptorSet>();
        writeDescriptorSet.descriptorCount = 1;
        writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        writeDescriptorSet.dstSet = descriptorSet;
        writeDescriptorSet.pBufferInfo = &descriptorBufferInfo;
        vkUpdateDescriptorSets(context.get_devices()[0], 1, &writeDescriptorSet, 0, nullptr);
    }

    void setup_physics_resources(btCollisionShape* pBtCollisionShape, float mass, const glm::vec3& initialPosition)
    {
        mInitialPosition = initialPosition;;
        dst::physics::RigidBody::CreateInfo rigidBodyCreateInfo { };
        rigidBodyCreateInfo.mass = mass;
        rigidBodyCreateInfo.initialTransform.setOrigin({ mInitialPosition.x, mInitialPosition.y, mInitialPosition.z });
        rigidBodyCreateInfo.pCollisionShape = pBtCollisionShape;
        dst::physics::RigidBody::create(&rigidBodyCreateInfo, &rigidBody);
        rigidBody.mupRigidBody->setUserPointer(this);
    }

    void setup_physics_resources(dst::physics::RigidBody::CreateInfo rigidBodyCreateInfo)
    {
#if 0
        mInitialPosition = initialPosition;;
        dst::physics::RigidBody::CreateInfo rigidBodyCreateInfo { };
        rigidBodyCreateInfo.mass = mass;
        rigidBodyCreateInfo.initialTransform.setOrigin({ mInitialPosition.x, mInitialPosition.y, mInitialPosition.z });
        rigidBodyCreateInfo.pCollisionShape = pBtCollisionShape;
        dst::physics::RigidBody::create(&rigidBodyCreateInfo, &rigidBody);
        rigidBody.mupRigidBody->setUserPointer(this);
#else

#endif
        rigidBodyCreateInfo.pUserData = this;
        dst::physics::RigidBody::create(&rigidBodyCreateInfo, &rigidBody);

    }

    void update_uniform_buffer(const gvk::Device& device)
    {
        ObjectUniforms ubo { };
        btTransform transform { };
        if (!isDynamic) {
            rigidBody.mupMotionState->setWorldTransform(rigidBody.mupRigidBody->getCenterOfMassTransform());
        }
        rigidBody.mupMotionState->getWorldTransform(transform);
        transform.getOpenGLMatrix(&ubo.world[0][0]);
        ubo.color = mColor;
        VmaAllocationInfo allocationInfo { };
        vmaGetAllocationInfo(device.get<VmaAllocator>(), mUniformBuffer.get<VmaAllocation>(), &allocationInfo);
        assert(allocationInfo.pMappedData);
        memcpy(allocationInfo.pMappedData, &ubo, sizeof(ObjectUniforms));
    }

    void record_cmds(const gvk::CommandBuffer& commandBuffer, const gvk::PipelineLayout& pipelineLayout)
    {
        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 1, 1, &(const VkDescriptorSet&)mDescriptorSet, 0, nullptr);
        mMesh.record_cmds(commandBuffer);
    }

    void set_color(const glm::vec4& color)
    {
        mColor = color;
    }

    dst::physics::RigidBody rigidBody;
    bool isDynamic { false };

private:
    glm::vec3 mInitialPosition { };
    glm::vec4 mColor { };
    gvk::Mesh mMesh;
    gvk::Buffer mUniformBuffer;
    gvk::DescriptorSet mDescriptorSet;
};

class MotionState final
    : public btMotionState
{
public:
    MotionState(btRigidBody* pRigidBody)
        : mpRigidBody { pRigidBody }
    {
        assert(mpRigidBody);
    }

    void getWorldTransform(btTransform& transform) const override final
    {
        transform = mTransform;
    }

    virtual void setWorldTransform(const btTransform& transform) override final
    {
        mTransform = transform;
    }

private:
    btTransform mTransform { btTransform::getIdentity() };
    btRigidBody* mpRigidBody { nullptr };
};

enum class State
{
    Attract,
    Intro,
    Idle,
    Play,
    Win,
    Celebration,
    Lose,
    Resetting,
    GameOver,
};

enum class ObjectType
{
    Unknown,
    Brick,
    Ball,
    Paddle,
    Ceiling,
    Wall,
    Floor,
};

static void bullet_physics_tick_callback(btDynamicsWorld* pDynamicsWorld, btScalar timeStep)
{
    (void)timeStep;
    assert(pDynamicsWorld);
    auto pCollisions = (std::set<std::pair<uint64_t, uint64_t>>*)pDynamicsWorld->getWorldUserInfo();
    assert(pCollisions);
    auto pDispatcher = pDynamicsWorld->getDispatcher();
    assert(pDispatcher);
    auto numManifolds = pDispatcher->getNumManifolds();
    for (int manifold_i = 0; manifold_i < numManifolds; ++manifold_i) {
        auto pManifold = pDispatcher->getManifoldByIndexInternal(manifold_i);
        assert(pManifold);
        int numContacts = pManifold->getNumContacts();
        for (int contact_i = 0; contact_i < numContacts; ++contact_i) {
            const auto& contactPoint = pManifold->getContactPoint(contact_i);
            if (contactPoint.getDistance() < 0) {
                auto collision = std::make_pair((uint64_t)pManifold->getBody0(), (uint64_t)pManifold->getBody1());
                if (collision.second < collision.first) {
                    std::swap(collision.first, collision.second);
                }
                pCollisions->insert(collision);
            }
        }
    }
}

struct ResetState
{
    btQuaternion rotation { };
    btVector3 translation { };
};

int main(int, const char* [])
{
    const float CeilingWidth = 32;
    const float CeilingHeight = 1;
    const float CeilingDepth = 1;

    const float WallWidth = 1;
    const float WallHeight = 64;
    const float WallDepth = 1;

    const float FloorWidth = 1024;
    const float FloorHeight = 1;
    const float FloorDepth = 1024;

    const float BrickWidth = 2;
    const float BrickHeight = 1;
    const float BrickDepth = 1;
    const float BrickMass = 8;
    const uint32_t BrickCount = 60;

    const float PaddleWidth = 6;
    const float PaddleHeight = 1;
    const float PaddleDepth = 0.1f;

    const float BallRadius = 0.5f;
    const float BallMass = 1;
    const uint32_t BallCount = 3;

    GfxContext gfxContext;
    auto vkResult = GfxContext::create("dynamic-static - Brick Breaker", &gfxContext);
    assert(vkResult == VK_SUCCESS);
    auto gvkDevice = gfxContext.get_devices()[0];
    auto gvkQueue = gvk::get_queue_family(gvkDevice, 0).queues[0];

    auto systemSurfaceCreateInfo = gvk::get_default<gvk::system::Surface::CreateInfo>();
    systemSurfaceCreateInfo.pTitle = gfxContext.get_instance().get<VkInstanceCreateInfo>().pApplicationInfo->pApplicationName;
    gvk::system::Surface systemSurface;
    auto success = gvk::system::Surface::create(&systemSurfaceCreateInfo, &systemSurface);
    assert(success);

    auto wsiManagerCreateInfo = gvk::get_default<gvk::WsiManager::CreateInfo>();
    auto win32SurfaceCreateInfo = gvk::get_default<VkWin32SurfaceCreateInfoKHR>();
    win32SurfaceCreateInfo.hinstance = GetModuleHandle(NULL);
    win32SurfaceCreateInfo.hwnd = (HWND)systemSurface.get_hwnd();
    wsiManagerCreateInfo.pWin32SurfaceCreateInfoKHR = &win32SurfaceCreateInfo;
    wsiManagerCreateInfo.sampleCount = VK_SAMPLE_COUNT_64_BIT;
    wsiManagerCreateInfo.depthFormat = VK_FORMAT_D32_SFLOAT;
    wsiManagerCreateInfo.presentMode = VK_PRESENT_MODE_MAILBOX_KHR;
    wsiManagerCreateInfo.queueFamilyIndex = gvkQueue.get<VkDeviceQueueCreateInfo>().queueFamilyIndex;
    gvk::WsiManager wsiManager;
    vkResult = gvk::WsiManager::create(gvkDevice, &wsiManagerCreateInfo, nullptr, &wsiManager);
    assert(vkResult == VK_SUCCESS);

    dst::physics::Context physicsContext;
    dst::physics::Context::create(&physicsContext);
    auto pDynamicsWorld = physicsContext.get_dynamics_world();
    assert(pDynamicsWorld);
    std::set<std::pair<uint64_t, uint64_t>> collisions;
    pDynamicsWorld->setInternalTickCallback(bullet_physics_tick_callback, &collisions);

    ///////////////////////////////////////////////////////////////////////////////
    dst::physics::Collider::Pool colliderPool;
    ///////////////////////////////////////////////////////////////////////////////

    // TODO : Documentation
    gvk::spirv::ShaderInfo vertexShaderInfo {
        .language = gvk::spirv::ShadingLanguage::Glsl,
        .stage = VK_SHADER_STAGE_VERTEX_BIT,
        .lineOffset = __LINE__,
        .source = R"(
            #version 450

            layout(set = 0, binding = 0)
            uniform CameraUniformBuffer
            {
                mat4 view;
                mat4 projection;
            } camera;

            layout(set = 1, binding = 0)
            uniform ObjectUniformBuffer
            {
                mat4 world;
                vec4 color;
            } object;

            layout(location = 0) in vec3 vsPosition;

            out gl_PerVertex
            {
                vec4 gl_Position;
            };

            void main()
            {
                gl_Position = camera.projection * camera.view * object.world * vec4(vsPosition, 1);
            }
        )"
    };

    // TODO : Documentation
    gvk::spirv::ShaderInfo fragmentShaderInfo {
        .language = gvk::spirv::ShadingLanguage::Glsl,
        .stage = VK_SHADER_STAGE_FRAGMENT_BIT,
        .lineOffset = __LINE__,
        .source = R"(
            #version 450

            layout(set = 1, binding = 0)
            uniform ObjectUniformBuffer
            {
                mat4 world;
                vec4 color;
            } object;

            layout(location = 0) out vec4 fragColor;

            void main()
            {
                
                fragColor = object.color;
            }
        )"
    };
    gvk::Pipeline pipeline;
    vkResult = dst_sample_create_pipeline<glm::vec3>(
        wsiManager.get_render_pass(),
        VK_CULL_MODE_BACK_BIT,
#if 0
        VK_POLYGON_MODE_LINE,
#else
        VK_POLYGON_MODE_FILL,
#endif
        vertexShaderInfo,
        fragmentShaderInfo,
        &pipeline
    );
    assert(vkResult == VK_SUCCESS);

    bool showGui = false;
    gvk::gui::Renderer guiRenderer;
    vkResult = gvk::gui::Renderer::create(
        gvkDevice,
        gvkQueue,
        gfxContext.get_command_buffers()[0],
        wsiManager.get_render_pass(),
        nullptr,
        &guiRenderer
    );

    // TODO : Documentation
    auto descriptorPoolSize = gvk::get_default<VkDescriptorPoolSize>();
    descriptorPoolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    descriptorPoolSize.descriptorCount = BallCount + BrickCount + 5; // BallCount + BrickCount + 1 paddle + 1 ceiling + 2 walls + 1 camera
    auto descriptorPoolCreateInfo = gvk::get_default<VkDescriptorPoolCreateInfo>();
    descriptorPoolCreateInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    descriptorPoolCreateInfo.maxSets = descriptorPoolSize.descriptorCount;
    descriptorPoolCreateInfo.poolSizeCount = 1;
    descriptorPoolCreateInfo.pPoolSizes = &descriptorPoolSize;
    gvk::DescriptorPool descriptorPool;
    vkResult = gvk::DescriptorPool::create(gfxContext.get_devices()[0], &descriptorPoolCreateInfo, nullptr, &descriptorPool);
    assert(vkResult == VK_SUCCESS);

    // TODO : Documentation
    gvk::math::Camera camera;
    gvk::math::FreeCameraController cameraController;
    cameraController.set_camera(&camera);
    camera.nearPlane = 1.0f;
    camera.transform.translation.z = -64;
    gvk::Buffer cameraUniformBuffer;
    vkResult = dst_sample_create_uniform_buffer<CameraUniforms>(gfxContext, &cameraUniformBuffer);
    assert(vkResult == VK_SUCCESS);
    
    // TODO : Documentation
    const auto& descriptorSetLayouts = pipeline.get<gvk::PipelineLayout>().get<gvk::DescriptorSetLayouts>();
    assert(descriptorSetLayouts.size() == 2);
    const auto& cameraDescriptorSetLayout = (VkDescriptorSetLayout)descriptorSetLayouts[0];
    const auto& objectDescriptorSetLayout = (VkDescriptorSetLayout)descriptorSetLayouts[1];
    auto descriptorSetAllocateInfo = gvk::get_default<VkDescriptorSetAllocateInfo>();
    descriptorSetAllocateInfo.descriptorPool = descriptorPool;
    descriptorSetAllocateInfo.descriptorSetCount = 1;
    descriptorSetAllocateInfo.pSetLayouts = &cameraDescriptorSetLayout;
    gvk::DescriptorSet cameraDescriptorSet;
    vkResult = gvk::DescriptorSet::allocate(gfxContext.get_devices()[0], &descriptorSetAllocateInfo, &cameraDescriptorSet);
    assert(vkResult == VK_SUCCESS);

    // TODO : Documentation
    auto descriptorBufferInfo = gvk::get_default<VkDescriptorBufferInfo>();
    descriptorBufferInfo.buffer = cameraUniformBuffer;
    auto writeDescriptorSet = gvk::get_default<VkWriteDescriptorSet>();
    writeDescriptorSet.descriptorCount = 1;
    writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writeDescriptorSet.dstSet = cameraDescriptorSet;
    writeDescriptorSet.pBufferInfo = &descriptorBufferInfo;
    vkUpdateDescriptorSets(gfxContext.get_devices()[0], 1, &writeDescriptorSet, 0, nullptr);

    std::map<uint64_t, btVector3> initialPositions;

    // TODO : Documentation
    descriptorSetAllocateInfo.pSetLayouts = &objectDescriptorSetLayout;

    // TODO : Documentation
    gvk::Mesh ceilingMesh;
    vkResult = create_box_mesh(gfxContext, { CeilingWidth, CeilingHeight, CeilingDepth }, &ceilingMesh);
    assert(vkResult == VK_SUCCESS);
    Object ceiling;
    {
        gvk::DescriptorSet descriptorSet;
        vkResult = gvk::DescriptorSet::allocate(gfxContext.get_devices()[0], &descriptorSetAllocateInfo, &descriptorSet);
        assert(vkResult == VK_SUCCESS);
        ceiling.setup_graphics_resources(gfxContext, ceilingMesh, descriptorSet, gvk::math::Color::White);
#if 0
        ceiling.setup_physics_resources(colliderPool.get_box_collider(btVector3(CeilingWidth, CeilingHeight, CeilingDepth) * 0.5f), 0, { 0, 32, 0 });
        ceiling.rigidBody.mupRigidBody->setLinearFactor(btVector3(1, 1, 0));
        ceiling.rigidBody.mupRigidBody->setRestitution(0.6f);
#endif
        dst::physics::RigidBody::CreateInfo rigidBodyCreateInfo { };
        // rigidBodyCreateInfo.linearFactor = { 1, 1, 0 };
        rigidBodyCreateInfo.material.restitution = 0.6f;
        rigidBodyCreateInfo.initialTransform.setOrigin({ 0, 32, 0 });
        rigidBodyCreateInfo.pCollisionShape = colliderPool.get_box_collider(btVector3(CeilingWidth, CeilingHeight, CeilingDepth) * 0.5f);
        ceiling.setup_physics_resources(rigidBodyCreateInfo);

        physicsContext.mupWorld->addRigidBody(ceiling.rigidBody.mupRigidBody.get(), AllGroup, AllGroup);
    }

    // TODO : Documentation
    Object floor;
    {
#if 0
        floor.setup_physics_resources(colliderPool.get_box_collider(btVector3(FloorWidth, FloorHeight, FloorDepth) * 0.5f), 0, { 0, -38, 0 });
        floor.rigidBody.mupRigidBody->setLinearFactor(btVector3(1, 1, 0));
#endif
        dst::physics::RigidBody::CreateInfo rigidBodyCreateInfo { };
        rigidBodyCreateInfo.initialTransform.setOrigin({ 0, -38, 0 });
        rigidBodyCreateInfo.pCollisionShape = colliderPool.get_box_collider(btVector3(FloorWidth, FloorHeight, FloorDepth) * 0.5f);
        floor.setup_physics_resources(rigidBodyCreateInfo);

        physicsContext.mupWorld->addRigidBody(floor.rigidBody.mupRigidBody.get(), AllGroup, AllGroup);
    }

    // TODO : Documentation
    gvk::Mesh wallMesh;
    vkResult = create_box_mesh(gfxContext, { WallWidth, WallHeight, WallDepth }, &wallMesh);
    assert(vkResult == VK_SUCCESS);
    Object leftWall;
    {
        gvk::DescriptorSet descriptorSet;
        vkResult = gvk::DescriptorSet::allocate(gfxContext.get_devices()[0], &descriptorSetAllocateInfo, &descriptorSet);
        assert(vkResult == VK_SUCCESS);
        leftWall.setup_graphics_resources(gfxContext, wallMesh, descriptorSet, gvk::math::Color::White);

#if 0
        leftWall.setup_physics_resources(colliderPool.get_box_collider(btVector3(WallWidth, WallHeight, WallDepth) * 0.5f), 0, { 16, 0, 0 });
        leftWall.rigidBody.mupRigidBody->setLinearFactor(btVector3(1, 1, 0));
        leftWall.rigidBody.mupRigidBody->setRestitution(0.6f);
#endif
        dst::physics::RigidBody::CreateInfo rigidBodyCreateInfo { };
        rigidBodyCreateInfo.material.restitution = 0.6f;
        rigidBodyCreateInfo.initialTransform.setOrigin({ 16, 0, 0 });
        rigidBodyCreateInfo.pCollisionShape = colliderPool.get_box_collider(btVector3(WallWidth, WallHeight, WallDepth) * 0.5f);
        leftWall.setup_physics_resources(rigidBodyCreateInfo);

        physicsContext.mupWorld->addRigidBody(leftWall.rigidBody.mupRigidBody.get(), AllGroup, AllGroup);
    }
    Object rightWall;
    {
        gvk::DescriptorSet descriptorSet;
        vkResult = gvk::DescriptorSet::allocate(gfxContext.get_devices()[0], &descriptorSetAllocateInfo, &descriptorSet);
        assert(vkResult == VK_SUCCESS);
        rightWall.setup_graphics_resources(gfxContext, wallMesh, descriptorSet, gvk::math::Color::White);

#if 0
        rightWall.setup_physics_resources(colliderPool.get_box_collider(btVector3(WallWidth, WallHeight, WallDepth) * 0.5f), 0, { -16, 0, 0 });
        rightWall.rigidBody.mupRigidBody->setLinearFactor(btVector3(1, 1, 0));
        rightWall.rigidBody.mupRigidBody->setRestitution(0.6f);
#endif
        dst::physics::RigidBody::CreateInfo rigidBodyCreateInfo { };
        rigidBodyCreateInfo.material.restitution = 0.6f;
        rigidBodyCreateInfo.initialTransform.setOrigin({ -16, 0, 0 });
        rigidBodyCreateInfo.pCollisionShape = colliderPool.get_box_collider(btVector3(WallWidth, WallHeight, WallDepth) * 0.5f);
        rightWall.setup_physics_resources(rigidBodyCreateInfo);

        physicsContext.mupWorld->addRigidBody(rightWall.rigidBody.mupRigidBody.get(), AllGroup, AllGroup);
    }

    // TODO : Documentation
    const uint32_t BrickRowCount = 6;
    const uint32_t BricksPerRow = 10;
    const std::array<glm::vec4, BrickRowCount> BrickRowColors {
        gvk::math::Color::Red,
        gvk::math::Color::Orange,
        gvk::math::Color::Yellow,
        gvk::math::Color::Green,
        gvk::math::Color::DodgerBlue,
        gvk::math::Color::Violet,
    };
    gvk::Mesh brickMesh;
    vkResult = create_box_mesh(gfxContext, { BrickWidth, BrickHeight, BrickDepth }, &brickMesh);
    assert(vkResult == VK_SUCCESS);
    std::array<Object, BrickRowCount * BricksPerRow> bricks;
    auto playAreaWidth = CeilingWidth - WallWidth;
    auto brickAreaWidth = playAreaWidth / BricksPerRow;
    std::set<uint64_t> liveBricks;
    for (size_t row_i = 0; row_i < BrickRowColors.size(); ++row_i) {
        auto offset = -playAreaWidth * 0.5f + brickAreaWidth * 0.5f;
        for (size_t brick_i = 0; brick_i < BricksPerRow; ++brick_i) {
            auto& brick = bricks[row_i * BricksPerRow + brick_i];

            btVector3 initialPosition(offset, 30.0f - row_i * BrickHeight * 2.0f, 0);

            gvk::DescriptorSet descriptorSet;
            vkResult = gvk::DescriptorSet::allocate(gfxContext.get_devices()[0], &descriptorSetAllocateInfo, &descriptorSet);
            assert(vkResult == VK_SUCCESS);
            brick.setup_graphics_resources(gfxContext, brickMesh, descriptorSet, BrickRowColors[row_i]);

#if 0
            brick.setup_physics_resources(colliderPool.get_box_collider(btVector3(BrickWidth, BrickHeight, BrickDepth) * 0.5f), BrickMass, { initialPosition.x(), initialPosition.y(), initialPosition.z() });
#endif
            dst::physics::RigidBody::CreateInfo rigidBodyCreateInfo { };
            rigidBodyCreateInfo.mass = BrickMass;
            rigidBodyCreateInfo.initialTransform.setOrigin(initialPosition);
            rigidBodyCreateInfo.pCollisionShape = colliderPool.get_box_collider(btVector3(BrickWidth, BrickHeight, BrickDepth) * 0.5f);
            brick.setup_physics_resources(rigidBodyCreateInfo);



            offset += brickAreaWidth;

            // brick.rigidBody.mupRigidBody->setLinearFactor(btVector3(1, 1, 0));
            // brick.rigidBody.mupRigidBody->setActivationState(0);

            ///////////////////////////////////////////////////////////////////////////////
            // physicsContext.mupWorld->removeRigidBody(brick.rigidBody.mupRigidBody.get());
            physicsContext.mupWorld->addCollisionObject(brick.rigidBody.mupRigidBody.get());
            ///////////////////////////////////////////////////////////////////////////////

            liveBricks.insert((uint64_t)brick.rigidBody.mupRigidBody.get());
            initialPositions.insert({ (uint64_t)brick.rigidBody.mupRigidBody.get(), initialPosition });

            
        }
    }

    // TODO : Documentation
    gvk::Mesh ballMesh;
    vkResult = create_sphere_mesh(gfxContext, BallRadius, 3, &ballMesh);
    assert(vkResult == VK_SUCCESS);
    std::array<Object, BallCount> balls;
    std::set<uint64_t> liveBalls;
    for (size_t i = 0; i < balls.size(); ++i) {
        auto& ball = balls[i];

        btVector3 initialPosition(-16.0f + i * 2.0f, 34, 0);

        gvk::DescriptorSet descriptorSet;
        vkResult = gvk::DescriptorSet::allocate(gfxContext.get_devices()[0], &descriptorSetAllocateInfo, &descriptorSet);
        assert(vkResult == VK_SUCCESS);
        ball.setup_graphics_resources(gfxContext, ballMesh, descriptorSet, gvk::math::Color::SlateGray);
#if 0
        ball.setup_physics_resources(colliderPool.get_sphere_collider(BallRadius), BallMass, { initialPosition.x(), initialPosition.y(), initialPosition.z() });
        ball.rigidBody.mupRigidBody->setLinearFactor(btVector3(1, 1, 0));
        ball.rigidBody.mupRigidBody->setRestitution(0.9f);
#endif
        dst::physics::RigidBody::CreateInfo rigidBodyCreateInfo { };
        rigidBodyCreateInfo.mass = BallMass;
        rigidBodyCreateInfo.material.restitution = 0.9f;
        rigidBodyCreateInfo.linearFactor = { 1, 1, 0 };
        rigidBodyCreateInfo.initialTransform.setOrigin(initialPosition);
        rigidBodyCreateInfo.pCollisionShape = colliderPool.get_sphere_collider(BallRadius);
        ball.setup_physics_resources(rigidBodyCreateInfo);

        liveBalls.insert((uint64_t)ball.rigidBody.mupRigidBody.get());
        initialPositions.insert({ (uint64_t)ball.rigidBody.mupRigidBody.get(), initialPosition });
    }

    // TODO : Documentation
    gvk::Mesh paddleMesh;
    vkResult = create_box_mesh(gfxContext, { PaddleWidth, PaddleHeight, PaddleDepth }, &paddleMesh);
    assert(vkResult == VK_SUCCESS);
    Object paddle;
    {
        gvk::DescriptorSet descriptorSet;
        vkResult = gvk::DescriptorSet::allocate(gfxContext.get_devices()[0], &descriptorSetAllocateInfo, &descriptorSet);
        assert(vkResult == VK_SUCCESS);
        paddle.setup_graphics_resources(gfxContext, paddleMesh, descriptorSet, gvk::math::Color::Brown);
#if 0
        paddle.setup_physics_resources(colliderPool.get_box_collider(btVector3(PaddleWidth, PaddleHeight, PaddleDepth) * 0.5f), 1, { 0, -28, 0 });
        paddle.rigidBody.mupRigidBody->setLinearFactor({ 1, 0, 0 });
        paddle.rigidBody.mupRigidBody->setAngularFactor({ 0, 0, 0 });
        paddle.rigidBody.mupRigidBody->setDamping(0.4f, 0.0f);
#endif
        dst::physics::RigidBody::CreateInfo rigidBodyCreateInfo { };
        rigidBodyCreateInfo.mass = 1;
        rigidBodyCreateInfo.linearDamping = 0.4f;
        rigidBodyCreateInfo.linearFactor = { 1, 0, 0 };
        rigidBodyCreateInfo.angularFactor = { 0, 0, 0 };
        rigidBodyCreateInfo.initialTransform.setOrigin({ 0, -28, 0 });
        rigidBodyCreateInfo.pCollisionShape = colliderPool.get_box_collider(btVector3(PaddleWidth, PaddleHeight, PaddleDepth) * 0.5f);
        paddle.setup_physics_resources(rigidBodyCreateInfo);

        paddle.isDynamic = true;
        physicsContext.mupWorld->addRigidBody(paddle.rigidBody.mupRigidBody.get());
    }

    float celebrationTimer = 0;
    float celebrationDuration = 4.5f;
    float celebrationColorDuration = 0.01f;
    std::vector<glm::vec4> celebrationColors {
        gvk::math::Color::Red,
        gvk::math::Color::White,
        gvk::math::Color::Blue,
        gvk::math::Color::Yellow
    };

    float resetTimer = 0;
    float resetDuration = 2.5f;
    std::map<uint64_t, ResetState> resetStates;

    float PaddleSpeed = 48;

    uint32_t ballCount = BallCount;
    State state = State::Play;
    // double frameTimeAccumulator = 0;
    gvk::system::Clock clock;
    while (
        !(systemSurface.get_input().keyboard.down(gvk::system::Key::Escape)) &&
        !(systemSurface.get_status() & gvk::system::Surface::CloseRequested)) {
        clock.update();
        gvk::system::Surface::update();
        const auto& input = systemSurface.get_input();

        if (input.keyboard.pressed(gvk::system::Key::One)) {
            useMotionState = !useMotionState;
            std::cout << "Use Motion State : " << useMotionState << std::endl;
        }

        // TODO : Documentation
        // if (!ImGui::GetIO().WantCaptureMouse && !ImGui::GetIO().WantCaptureKeyboard) {
            if (input.keyboard.down(gvk::system::Key::LeftArrow)) {
                paddle.rigidBody.mupRigidBody->activate(true);
                paddle.rigidBody.mupRigidBody->applyCentralForce(btVector3 { PaddleSpeed, 0, 0 });
            }
            if (input.keyboard.down(gvk::system::Key::RightArrow)) {
                paddle.rigidBody.mupRigidBody->activate(true);
                paddle.rigidBody.mupRigidBody->applyCentralForce(btVector3 { -PaddleSpeed, 0, 0 });
            }
        // }
        switch (state) {
        case State::Attract:
            {

            } break;
            case State::Intro:
            {

            } break;
            case State::Idle:
            {

            } break;
            case State::Play:
            {
                if (!ImGui::GetIO().WantCaptureMouse && !ImGui::GetIO().WantCaptureKeyboard) {
                    if (input.keyboard.pressed(gvk::system::Key::SpaceBar)) {
                        if (ballCount) {
                            assert(ballCount <= balls.size());
                            auto& ball = balls[ballCount - 1];
                            ball.isDynamic = true;
                            ball.rigidBody.mupRigidBody->activate(true);
                            ball.rigidBody.mupRigidBody->setCenterOfMassTransform(btTransform::getIdentity());
                            physicsContext.mupWorld->addRigidBody(ball.rigidBody.mupRigidBody.get());
                            ballCount -= 1;
                        }
                    }
                }

                // auto lv = balls[0].rigidBody.mupRigidBody->getLinearVelocity();
                // std::cout << lv.x() << " : " << lv.y() << " : " << lv.z() << std::endl;

                for (const auto& collision : collisions) {
                    for (auto collider : std::array<uint64_t, 2> { collision.first, collision.second }) {
                        if (liveBricks.count(collider)) {
                            liveBricks.erase(collider);

                            auto pRigidBody = (btRigidBody*)collider;
                            if (!((Object*)pRigidBody->getUserPointer())->isDynamic) {
                                ((Object*)pRigidBody->getUserPointer())->isDynamic = true;
                                physicsContext.mupWorld->removeRigidBody(pRigidBody);
                                
                                

                                // pRigidBody->getMotionState()->setWorldTransform(pRigidBody->getWorldTransform());

                                pRigidBody->getMotionState()->setWorldTransform(pRigidBody->getCenterOfMassTransform());
                                pRigidBody->forceActivationState(1);
                                pRigidBody->activate(true);

                                physicsContext.mupWorld->addRigidBody(pRigidBody); // , BrickGroup, AllGroup & ~PaddleGroup);

                            }

                            
                            
                            // btTransform transform { };
                            // pRigidBody->getWorldTransform(transform);
                            // // transform.getOrigin().setZ(2);
                            // pRigidBody->setWorldTransform(transform);
                            // pRigidBody->setCenterOfMassTransform(transform);
                            // pRigidBody->applyTorqueImpulse({ 16, 0, 0 });
                            // pRigidBody->activate(true);
                        }
                    }
                }

                for (auto& ball : balls) {
                    #if 0
                    for (const auto& brick : bricks) {
                        auto collision = std::make_pair((uint64_t)ball.rigidBody.mupRigidBody.get(), (uint64_t)brick.rigidBody.mupRigidBody.get());
                        if (collision.second < collision.first) {
                            std::swap(collision.first, collision.second);
                        }
                        if (collisions.count(collision)) {
                            if (paddle.rigidBody.mupRigidBody->getWorldTransform().getOrigin().y() < brick.rigidBody.mupRigidBody->getWorldTransform().getOrigin().y()) {
                                physicsContext.mupWorld->removeCollisionObject(brick.rigidBody.mupRigidBody.get());
                                physicsContext.mupWorld->addRigidBody(brick.rigidBody.mupRigidBody.get(), BrickGroup, AllGroup & ~PaddleGroup);
                                // btTransform transform { };
                                // brick.rigidBody.mupMotionState->getWorldTransform(transform);
                                // // transform.getOrigin().setZ(2);
                                // brick.rigidBody.mupMotionState->setWorldTransform(transform);
                                // brick.rigidBody.mupRigidBody->setCenterOfMassTransform(transform);
                                // brick.rigidBody.mupRigidBody->applyTorqueImpulse({ 16, 0, 0 });
                                brick.rigidBody.mupRigidBody->activate(true);
                            }
                        }
                    }
                    #endif
                    {
                        auto collision = std::make_pair((uint64_t)ball.rigidBody.mupRigidBody.get(), (uint64_t)paddle.rigidBody.mupRigidBody.get());
                        if (collision.second < collision.first) {
                            std::swap(collision.first, collision.second);
                        }
                        if (collisions.count(collision)) {
                            const auto& ballTransform = ball.rigidBody.mupRigidBody->getCenterOfMassTransform();
                            const auto& paddleTransform = paddle.rigidBody.mupRigidBody->getCenterOfMassTransform();
                            if (paddleTransform.getOrigin().y() < ballTransform.getOrigin().y()) {
                                auto impulse = (ballTransform.getOrigin() - paddleTransform.getOrigin()).normalized();
                                impulse *= 64;
                                impulse.setY(64);
                                ball.rigidBody.mupRigidBody->applyCentralImpulse(impulse);
                            }
                        }
                    }

                    auto paddleTransform = paddle.rigidBody.mupRigidBody->getCenterOfMassTransform();
                    auto ballTransform = ball.rigidBody.mupRigidBody->getCenterOfMassTransform();
                    auto floorTransform = floor.rigidBody.mupRigidBody->getCenterOfMassTransform();
                    if (ballTransform.getOrigin().y() <= (paddleTransform.getOrigin().y() + floorTransform.getOrigin().y()) * 0.5f) {
                        liveBalls.erase((uint64_t)ball.rigidBody.mupRigidBody.get());
                    }
                }

                if (liveBricks.empty()) {
                    state = State::Win;
                } else if (liveBalls.empty()) {
                    state = State::Lose;
                }
            } break;
            case State::Win:
            {
                celebrationTimer = 0;
                state = State::Celebration;
            } break;
            case State::Celebration:
            {
                celebrationTimer += clock.elapsed<gvk::system::Seconds<float>>();
                if (celebrationTimer < celebrationDuration) {
                    auto index = (size_t)std::round(celebrationTimer / celebrationColorDuration) % celebrationColors.size();
                    auto color = celebrationColors[index];
                    leftWall.set_color(color);
                    rightWall.set_color(color);
                    ceiling.set_color(color);
                } else {
                    state = State::Lose;
                    leftWall.set_color(gvk::math::Color::White);
                    rightWall.set_color(gvk::math::Color::White);
                    ceiling.set_color(gvk::math::Color::White);
                }
            } break;
            case State::Lose:
            {
                resetTimer = 0;
                resetStates.clear();
                state = State::Resetting;
                for (auto& brick : bricks) {
                    liveBricks.insert((uint64_t)brick.rigidBody.mupRigidBody.get());
                    brick.rigidBody.mupRigidBody->setLinearVelocity(btVector3(0, 0, 0));
                    brick.rigidBody.mupRigidBody->setAngularVelocity(btVector3(0, 0, 0));
                    physicsContext.get_dynamics_world()->removeRigidBody(brick.rigidBody.mupRigidBody.get());
                    brick.isDynamic = false;
                    auto transform = brick.rigidBody.mupRigidBody->getCenterOfMassTransform();
                    ResetState resetState { };
                    resetState.rotation = transform.getRotation();
                    resetState.translation = transform.getOrigin();
                    resetStates.insert({ (uint64_t)brick.rigidBody.mupRigidBody.get(), resetState });
                }
                for (auto& ball : balls) {
                    liveBalls.insert((uint64_t)ball.rigidBody.mupRigidBody.get());
                    ball.rigidBody.mupRigidBody->setLinearVelocity(btVector3(0, 0, 0));
                    ball.rigidBody.mupRigidBody->setAngularVelocity(btVector3(0, 0, 0));
                    physicsContext.get_dynamics_world()->removeRigidBody(ball.rigidBody.mupRigidBody.get());
                    ball.isDynamic = false;
                    auto transform = ball.rigidBody.mupRigidBody->getCenterOfMassTransform();
                    ResetState resetState { };
                    resetState.rotation = transform.getRotation();
                    resetState.translation = transform.getOrigin();
                    resetStates.insert({ (uint64_t)ball.rigidBody.mupRigidBody.get(), resetState });
                }
            } break;
            case State::Resetting:
            {
                resetTimer += clock.elapsed<gvk::system::Seconds<float>>();
                if (resetTimer < resetDuration) {
                    float t = resetTimer / resetDuration;
                    for (const auto& resetState : resetStates) {
                        auto pRigidBody = (btRigidBody*)resetState.first;
                        btTransform transform { };
                        transform = pRigidBody->getCenterOfMassTransform();
                        auto initialiPositionItr = initialPositions.find(resetState.first);
                        assert(initialiPositionItr != initialPositions.end());
                        transform.setOrigin(resetState.second.translation.lerp(initialiPositionItr->second, t));
                        transform.setRotation(resetState.second.rotation.slerp(btQuaternion::getIdentity(), t));
                        pRigidBody->setCenterOfMassTransform(transform);
                    }
                } else {
                    for (const auto& resetState : resetStates) {
                        auto pRigidBody = (btRigidBody*)resetState.first;
                        btTransform transform { };
                        transform = pRigidBody->getCenterOfMassTransform();
                        auto initialiPositionItr = initialPositions.find(resetState.first);
                        assert(initialiPositionItr != initialPositions.end());
                        transform.setOrigin(initialiPositionItr->second);
                        transform.setRotation(btQuaternion::getIdentity());
                        pRigidBody->setCenterOfMassTransform(transform);
                    }
                    for (const auto& brick : bricks) {
                        liveBricks.insert((uint64_t)brick.rigidBody.mupRigidBody.get());
                        brick.rigidBody.mupMotionState->setWorldTransform(brick.rigidBody.mupRigidBody->getCenterOfMassTransform());
                        physicsContext.mupWorld->addCollisionObject(brick.rigidBody.mupRigidBody.get());
                        // brick.rigidBody.mupRigidBody->setActivationState(0);
                    }
                    for (const auto& ball : balls) {
                        ball.rigidBody.mupMotionState->setWorldTransform(ball.rigidBody.mupRigidBody->getCenterOfMassTransform());
                        liveBalls.insert((uint64_t)ball.rigidBody.mupRigidBody.get());
                        // physicsContext.mupWorld->addRigidBody(ball.rigidBody.mupRigidBody.get());
                        // ball.rigidBody.mupRigidBody->setActivationState(0);
                    }
                    resetStates.clear();
                    ballCount = 3;
                    state = State::Play;
                }
            } break;
            case State::GameOver:
            {

            } break;
        }

        collisions.clear();
        auto deltaTime = clock.elapsed<gvk::system::Seconds<float>>();
        physicsContext.mupWorld->stepSimulation(deltaTime);

        if (!ImGui::GetIO().WantCaptureMouse && !ImGui::GetIO().WantCaptureKeyboard) {
            gvk::math::FreeCameraController::UpdateInfo cameraControllerUpdateInfo {
                .deltaTime = deltaTime,
                .moveUp = input.keyboard.down(gvk::system::Key::Q),
                .moveDown = input.keyboard.down(gvk::system::Key::E),
                .moveLeft = input.keyboard.down(gvk::system::Key::A),
                .moveRight = input.keyboard.down(gvk::system::Key::D),
                .moveForward = input.keyboard.down(gvk::system::Key::W),
                .moveBackward = input.keyboard.down(gvk::system::Key::S),
                .moveSpeedMultiplier = input.keyboard.down(gvk::system::Key::LeftShift) ? 2.0f : 1.0f,
                .lookDelta = { input.mouse.position.delta()[0], input.mouse.position.delta()[1] },
                .fieldOfViewDelta = input.mouse.scroll.delta()[1],
            };
            cameraController.lookEnabled = input.mouse.buttons.down(gvk::system::Mouse::Button::Left);
            if (input.mouse.buttons.pressed(gvk::system::Mouse::Button::Right)) {
                camera.fieldOfView = 60.0f;
            }
            cameraController.update(cameraControllerUpdateInfo);
        }

        // TODO : Documentation
        CameraUniforms cameraUbo { };
        cameraUbo.view = camera.view();
        cameraUbo.projection = camera.projection();
        VmaAllocationInfo allocationInfo { };
        vmaGetAllocationInfo(gfxContext.get_devices()[0].get<VmaAllocator>(), cameraUniformBuffer.get<VmaAllocation>(), &allocationInfo);
        assert(allocationInfo.pMappedData);
        memcpy(allocationInfo.pMappedData, &cameraUbo, sizeof(CameraUniforms));

        paddle.update_uniform_buffer(gfxContext.get_devices()[0]);
        ceiling.update_uniform_buffer(gfxContext.get_devices()[0]);
        leftWall.update_uniform_buffer(gfxContext.get_devices()[0]);
        rightWall.update_uniform_buffer(gfxContext.get_devices()[0]);
        for (auto& brick : bricks) {
            brick.update_uniform_buffer(gfxContext.get_devices()[0]);
        }
        for (auto& ball : balls) {
            ball.update_uniform_buffer(gfxContext.get_devices()[0]);
        }

        wsiManager.update();
        auto swapchain = wsiManager.get_swapchain();
        if (swapchain) {
            uint32_t imageIndex = 0;
            vkResult = wsiManager.acquire_next_image(UINT64_MAX, VK_NULL_HANDLE, &imageIndex);
            assert(vkResult == VK_SUCCESS || vkResult == VK_SUBOPTIMAL_KHR);

            auto extent = wsiManager.get_swapchain().get<VkSwapchainCreateInfoKHR>().imageExtent;
            camera.set_aspect_ratio(extent.width, extent.height);

            const auto& vkFences = wsiManager.get_vk_fences();

            if (showGui) {
                auto imguiCursor = ImGui::GetMouseCursor();
                if (imguiCursor == ImGuiMouseCursor_None || ImGui::GetIO().MouseDrawCursor) {
                    systemSurface.set_cursor_mode(gvk::system::Surface::CursorMode::Hidden);
                } else {
                    switch (imguiCursor) {
                    case ImGuiMouseCursor_Arrow: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::Arrow); break;
                    case ImGuiMouseCursor_TextInput: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::IBeam); break;
                    case ImGuiMouseCursor_Hand: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::Hand); break;
                    case ImGuiMouseCursor_ResizeNS: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::ResizeNS); break;
                    case ImGuiMouseCursor_ResizeEW: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::ResizeEW); break;
                    case ImGuiMouseCursor_ResizeAll: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::ResizeAll); break;
                    case ImGuiMouseCursor_ResizeNESW: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::ResizeNESW); break;
                    case ImGuiMouseCursor_ResizeNWSE: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::ResizeNWSE); break;
                    case ImGuiMouseCursor_NotAllowed: systemSurface.set_cursor_type(gvk::system::Surface::CursorType::NotAllowed); break;
                    default: break;
                    }
                }
                if (systemSurface.get_status() & gvk::system::Surface::GainedFocus) {
                    ImGui::GetIO().AddFocusEvent(true);
                }
                if (systemSurface.get_status() & gvk::system::Surface::LostFocus) {
                    ImGui::GetIO().AddFocusEvent(false);
                }
                const auto& textStream = systemSurface.get_text_stream();
                auto guiRendererBeginInfo = gvk::get_default<gvk::gui::Renderer::BeginInfo>();
                guiRendererBeginInfo.deltaTime = deltaTime;
                guiRendererBeginInfo.extent = { (float)extent.width, (float)extent.height };
                guiRendererBeginInfo.pInput = &input;
                guiRendererBeginInfo.textStreamCodePointCount = (uint32_t)textStream.size();
                guiRendererBeginInfo.pTextStreamCodePoints = !textStream.empty() ? textStream.data() : nullptr;
                guiRenderer.begin_gui(guiRendererBeginInfo);
                ImGui::ShowDemoWindow();
                ImGui::DragFloat("Paddle Speed", &PaddleSpeed, 12, 100);
                auto paddleMass = paddle.rigidBody.mupRigidBody->getMass();
                if (ImGui::DragFloat("Paddle Mass", &paddleMass, 0.01f, 0.01f)) {
                    physicsContext.get_dynamics_world()->removeRigidBody(paddle.rigidBody.mupRigidBody.get());
                    btVector3 inertia { };
                    paddle.rigidBody.mupRigidBody->getCollisionShape()->calculateLocalInertia(paddleMass, inertia);
                    paddle.rigidBody.mupRigidBody->setMassProps(paddleMass, inertia);
                    physicsContext.get_dynamics_world()->addRigidBody(paddle.rigidBody.mupRigidBody.get(), PaddleGroup, AllGroup & ~BrickGroup);
                }
                #if 0
                ImGui::DragFloat("anchor", &anchor, 0.01f);
                ImGui::DragFloat("amplitude", &amplitude, 0.1f);
                ImGui::DragFloat("frequency", &frequency, 0.1f);
                ImGui::DragFloat("yRotation", &yRotation, 0.1f);
                ImGui::DragFloat("zRotation", &zRotation, 0.1f);
                ImGui::DragFloat("guiImageScale", &guiImageScale, 0.005f, 0.01f, 0.5f);
                ImVec2 guiImageExtent { renderTargetCreateInfo.extent.width * guiImageScale, renderTargetCreateInfo.extent.height * guiImageScale };
                ImGui::Image(guiDescriptorSets[0], guiImageExtent);
                #endif
                guiRenderer.end_gui((uint32_t)vkFences.size(), !vkFences.empty() ? vkFences.data() : nullptr);
            }

            vkResult = vkWaitForFences(gvkDevice, 1, &vkFences[imageIndex], VK_TRUE, UINT64_MAX);
            assert(vkResult == VK_SUCCESS);
            vkResult = vkResetFences(gvkDevice, 1, &vkFences[imageIndex]);
            assert(vkResult == VK_SUCCESS);

            const auto& commandBuffer = wsiManager.get_command_buffers()[imageIndex];
            vkResult = vkBeginCommandBuffer(commandBuffer, &gvk::get_default<VkCommandBufferBeginInfo>());
            assert(vkResult == VK_SUCCESS);
            auto renderPassBeginInfo = wsiManager.get_render_targets()[imageIndex].get_render_pass_begin_info();
            vkCmdBeginRenderPass(commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
            {
                // TODO : Documentation
                VkRect2D scissor { .extent = renderPassBeginInfo.renderArea.extent };
                vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
                VkViewport viewport { .width = (float)scissor.extent.width, .height = (float)scissor.extent.height, .minDepth = 0, .maxDepth = 1 };
                vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

                // TODO : Documentation
                vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

                // TODO : Documentation
                vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline.get<gvk::PipelineLayout>(), 0, 1, &(const VkDescriptorSet&)cameraDescriptorSet, 0, nullptr);

                // TODO : Documentation
                paddle.record_cmds(commandBuffer, pipeline.get<gvk::PipelineLayout>());
                ceiling.record_cmds(commandBuffer, pipeline.get<gvk::PipelineLayout>());
                leftWall.record_cmds(commandBuffer, pipeline.get<gvk::PipelineLayout>());
                rightWall.record_cmds(commandBuffer, pipeline.get<gvk::PipelineLayout>());
                for (auto& brick : bricks) {
                    brick.record_cmds(commandBuffer, pipeline.get<gvk::PipelineLayout>());
                }
                for (auto& ball : balls) {
                    ball.record_cmds(commandBuffer, pipeline.get<gvk::PipelineLayout>());
                }
            }

            if (showGui) {
                guiRenderer.record_cmds(commandBuffer);
            }

            vkCmdEndRenderPass(commandBuffer);
            vkResult = vkEndCommandBuffer(commandBuffer);
            assert(vkResult == VK_SUCCESS);

            auto submitInfo = wsiManager.get_submit_info(imageIndex);
            vkResult = vkQueueSubmit(gvkQueue, 1, &submitInfo, vkFences[imageIndex]);
            assert(vkResult == VK_SUCCESS);

            auto presentInfo = wsiManager.get_present_info(&imageIndex);
            vkResult = vkQueuePresentKHR(gvkQueue, &presentInfo);
            assert(vkResult == VK_SUCCESS || vkResult == VK_SUBOPTIMAL_KHR);
        }

        if (input.keyboard.pressed(gvk::system::Key::OEM_Tilde)) {
            showGui = !showGui;
        }

    }
    vkResult = vkDeviceWaitIdle(gfxContext.get_devices()[0]);
    assert(vkResult == VK_SUCCESS);

    //remove the rigidbodies from the dynamics world and delete them
    for (int i = physicsContext.mupWorld->getNumCollisionObjects() - 1; 0 <= i; --i) {
        btCollisionObject* obj = physicsContext.mupWorld->getCollisionObjectArray()[i];
        // btRigidBody* body = btRigidBody::upcast(obj);
        physicsContext.mupWorld->removeCollisionObject(obj);
    }
    return 0;
}
