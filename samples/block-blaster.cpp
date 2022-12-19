
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
#include "dynamic-static.graphics/primitives.hpp"
#include "dynamic-static/finite-state-machine.hpp"

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

VkResult create_sphere_mesh(const gvk::Context& context, float radius, uint32_t subdivisions, gvk::Mesh* pMesh)
{
    std::vector<glm::vec3> vertices(dst::gfx::primitive::Icosahedron::Vertices.begin(), dst::gfx::primitive::Icosahedron::Vertices.end());
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
        &triangles[0][0]
    );
}

VkResult create_cube_mesh(const gvk::Context& context, gvk::Mesh* pMesh)
{
    return pMesh->write(
        context.get_devices()[0],
        gvk::get_queue_family(context.get_devices()[0], 0).queues[0],
        context.get_command_buffers()[0],
        VK_NULL_HANDLE,
        (uint32_t)dst::gfx::primitive::Cube::Vertices.size(),
        dst::gfx::primitive::Cube::Vertices.data(),
        (uint32_t)dst::gfx::primitive::Cube::Triangles.size() * 3,
        dst::gfx::primitive::Cube::Triangles.data()
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

class RigidBody final
{
public:
    struct CreateInfo
    {
        float mass { };
        btVector3 initialPosition { };
        btCollisionShape* pCollisionShape { nullptr };
    };

    static void create(dst::physics::Context& physicsContext, const CreateInfo* pCreateInfo, RigidBody* pRigidBody)
    {
        assert(pCreateInfo);
        assert(pCreateInfo->pCollisionShape);
        assert(pRigidBody);
        btTransform transform { };
        transform.setIdentity();
        transform.setOrigin(pCreateInfo->initialPosition);
        pRigidBody->mupMotionState = std::make_unique<btDefaultMotionState>(transform);
        btVector3 inertia { };
        if (pCreateInfo->mass) {
            pCreateInfo->pCollisionShape->calculateLocalInertia(pCreateInfo->mass, inertia);
        }
        pRigidBody->mupRigidBody = std::make_unique<btRigidBody>(
            btRigidBody::btRigidBodyConstructionInfo(pCreateInfo->mass, pRigidBody->mupMotionState.get(), pCreateInfo->pCollisionShape, inertia)
        );
        physicsContext.mupWorld->addRigidBody(pRigidBody->mupRigidBody.get());
    }

    btTransform get_bt_transform() const
    {
        btTransform btTransform { };
        mupMotionState->getWorldTransform(btTransform);
        return btTransform;
    }

    std::unique_ptr<btMotionState> mupMotionState;
    std::unique_ptr<btRigidBody> mupRigidBody;
};

class Object final
{
public:
    void setup_graphics_resources(const gvk::Context& context, const gvk::Mesh& mesh, const gvk::DescriptorSet& descriptorSet, const glm::vec4& color, const glm::vec3& scale = { 1, 1, 1 })
    {
        mTransform.scale = scale;
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

    void setup_physics_resources(dst::physics::Context& context, btCollisionShape* pBtCollisionShape, float mass, const glm::vec3& position)
    {
        mTransform.translation = position;;
        RigidBody::CreateInfo rigidBodyCreateInfo { };
        rigidBodyCreateInfo.mass = mass;
        rigidBodyCreateInfo.initialPosition.setValue(position.x, position.y, position.z);
        rigidBodyCreateInfo.pCollisionShape = pBtCollisionShape;
        RigidBody::create(context, &rigidBodyCreateInfo, &mRigidBody);
        mRigidBody.mupRigidBody->setLinearFactor(btVector3(1, 1, 0));
        mRigidBody.mupRigidBody->setActivationState(0);
    }

    void update_uniform_buffer(const gvk::Device& device)
    {
        ObjectUniforms ubo { };
        mRigidBody.get_bt_transform().getOpenGLMatrix(&ubo.world[0][0]);
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

private:
    gvk::math::Transform mTransform { };
    glm::vec4 mColor { };
    gvk::Mesh mMesh;
    gvk::Buffer mUniformBuffer;
    gvk::DescriptorSet mDescriptorSet;
    RigidBody mRigidBody;
};

class Game final
{
public:
    enum class State
    {
        Attract,
        Intro,
        Idle,
        Play,
        Win,
        Lose,
        GameOver,
    };

    State state { State::Attract };
    std::vector<Object> walls;
    std::vector<Object> balls;
    std::vector<Object> bricks;
    Object paddle;
};

int main(int, const char* [])
{
    const float CeilingWidth = 32;
    const float CeilingHeight = 1;
    const float CeilingDepth = 1;

    const float WallWidth = 1;
    const float WallHeight = 64;
    const float WallDepth = 1;

    const float BrickWidth = 1;
    const float BrickHeight = 1;
    const float BrickDepth = 1;
    const uint32_t BrickCount = 4;

    const float PaddleWidth = 4;
    const float PaddleHeight = 1;
    const float PaddleDepth = 1;

    const float BallRadius = 1;
    const float BallMass = 1;
    const uint32_t BallCount = 3;



    Game game;

    dst::physics::Context physicsContext;
    dst::physics::Context::create(&physicsContext);

    btBoxShape ceilingCollisionShape(btVector3(CeilingWidth, CeilingHeight, CeilingDepth) * 0.5f);
    btBoxShape wallCollisionShape(btVector3(WallWidth, WallHeight, WallDepth) * 0.5f);
    btBoxShape brickCollisionShape(btVector3(BrickWidth, BrickHeight, BrickDepth) * 0.5f);
    btBoxShape paddleCollisionShape(btVector3(PaddleWidth, PaddleHeight, PaddleDepth) * 0.5f);
    btSphereShape ballCollisionShape(BallRadius);

    GfxContext gfxContext;
    auto vkResult = GfxContext::create("dynamic-static - Brick Breaker", &gfxContext);
    assert(vkResult == VK_SUCCESS);

    // TODO : Documentation
    gvk::Mesh cubeMesh;
    vkResult = create_cube_mesh(gfxContext, &cubeMesh);
    assert(vkResult == VK_SUCCESS);
    gvk::Mesh sphereMesh;
    vkResult = create_sphere_mesh(gfxContext, BallRadius, 3, &sphereMesh);

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
        gfxContext.get_wsi_manager().get_render_pass(),
        VK_CULL_MODE_BACK_BIT,
        VK_POLYGON_MODE_LINE,
        vertexShaderInfo,
        fragmentShaderInfo,
        &pipeline
    );
    assert(vkResult == VK_SUCCESS);

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

    // TODO : Documentation
    descriptorSetAllocateInfo.pSetLayouts = &objectDescriptorSetLayout;

    // TODO : Documentation
    Object paddle;
    {
        gvk::DescriptorSet descriptorSet;
        vkResult = gvk::DescriptorSet::allocate(gfxContext.get_devices()[0], &descriptorSetAllocateInfo, &descriptorSet);
        assert(vkResult == VK_SUCCESS);
        paddle.setup_graphics_resources(gfxContext, cubeMesh, descriptorSet, gvk::math::Color::AntiqueWhite);
        paddle.setup_physics_resources(physicsContext, &paddleCollisionShape, 0, { 0, 0, 16 });
    }

    // TODO : Documentation
    Object ceiling;
    {
        gvk::DescriptorSet descriptorSet;
        vkResult = gvk::DescriptorSet::allocate(gfxContext.get_devices()[0], &descriptorSetAllocateInfo, &descriptorSet);
        assert(vkResult == VK_SUCCESS);
        ceiling.setup_graphics_resources(gfxContext, cubeMesh, descriptorSet, gvk::math::Color::AntiqueWhite);
        ceiling.setup_physics_resources(physicsContext, &ceilingCollisionShape, 0, { 0, 32, 0 });
    }

    // TODO : Documentation
    Object leftWall;
    {
        gvk::DescriptorSet descriptorSet;
        vkResult = gvk::DescriptorSet::allocate(gfxContext.get_devices()[0], &descriptorSetAllocateInfo, &descriptorSet);
        assert(vkResult == VK_SUCCESS);
        leftWall.setup_graphics_resources(gfxContext, cubeMesh, descriptorSet, gvk::math::Color::AntiqueWhite);
        leftWall.setup_physics_resources(physicsContext, &wallCollisionShape, 0, { 16, 0, 0 });
    }

    // TODO : Documentation
    Object rightWall;
    {
        gvk::DescriptorSet descriptorSet;
        vkResult = gvk::DescriptorSet::allocate(gfxContext.get_devices()[0], &descriptorSetAllocateInfo, &descriptorSet);
        assert(vkResult == VK_SUCCESS);
        rightWall.setup_graphics_resources(gfxContext, cubeMesh, descriptorSet, gvk::math::Color::AntiqueWhite);
        rightWall.setup_physics_resources(physicsContext, &wallCollisionShape, 0, { 0, 0, 16 });
    }

    // TODO : Documentation
    std::array<Object, BallCount> balls;
    for (size_t i = 0; i < balls.size(); ++i) {
        auto& ball = balls[i];
        gvk::DescriptorSet descriptorSet;
        vkResult = gvk::DescriptorSet::allocate(gfxContext.get_devices()[0], &descriptorSetAllocateInfo, &descriptorSet);
        assert(vkResult == VK_SUCCESS);
        ball.setup_graphics_resources(gfxContext, sphereMesh, descriptorSet, gvk::math::Color::Plum);
        ball.setup_physics_resources(physicsContext, &ballCollisionShape, BallMass, { 0, 0, 0 });
    }

    // TODO : Documentation
    std::array<Object, BrickCount> bricks;
    for (size_t i = 0; i < bricks.size(); ++i) {
        auto& brick = bricks[i];
        gvk::DescriptorSet descriptorSet;
        vkResult = gvk::DescriptorSet::allocate(gfxContext.get_devices()[0], &descriptorSetAllocateInfo, &descriptorSet);
        assert(vkResult == VK_SUCCESS);
        brick.setup_graphics_resources(gfxContext, sphereMesh, descriptorSet, gvk::math::Color::Orange);
        brick.setup_physics_resources(physicsContext, &ballCollisionShape, BallMass, { 0, 0, 0 });
    }

    gvk::sys::Clock clock;
    while (
        !(gfxContext.get_sys_surface().get_input().keyboard.down(gvk::sys::Key::Escape)) &&
        !(gfxContext.get_sys_surface().get_status() & gvk::sys::Surface::CloseRequested)) {
        gvk::sys::Surface::update();
        clock.update();

        auto deltaTime = clock.elapsed<gvk::sys::Seconds<float>>();
        const auto& input = gfxContext.get_sys_surface().get_input();
        gvk::math::FreeCameraController::UpdateInfo cameraControllerUpdateInfo {
            .deltaTime = deltaTime,
            .moveUp = input.keyboard.down(gvk::sys::Key::Q),
            .moveDown = input.keyboard.down(gvk::sys::Key::E),
            .moveLeft = input.keyboard.down(gvk::sys::Key::A),
            .moveRight = input.keyboard.down(gvk::sys::Key::D),
            .moveForward = input.keyboard.down(gvk::sys::Key::W),
            .moveBackward = input.keyboard.down(gvk::sys::Key::S),
            .moveSpeedMultiplier = input.keyboard.down(gvk::sys::Key::LeftShift) ? 2.0f : 1.0f,
            .lookDelta = { input.mouse.position.delta()[0], input.mouse.position.delta()[1] },
            .fieldOfViewDelta = input.mouse.scroll.delta()[1],
        };
        cameraController.lookEnabled = input.mouse.buttons.down(gvk::sys::Mouse::Button::Left);
        if (input.mouse.buttons.pressed(gvk::sys::Mouse::Button::Right)) {
            camera.fieldOfView = 60.0f;
        }
        cameraController.update(cameraControllerUpdateInfo);

        // TODO : Documentation
        physicsContext.mupWorld->stepSimulation(deltaTime, 10);

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
        for (auto& ball : balls) {
            ball.update_uniform_buffer(gfxContext.get_devices()[0]);
        }

        auto& wsiManager = gfxContext.get_wsi_manager();
        if (wsiManager.update()) {
            auto extent = wsiManager.get_swapchain().get<VkSwapchainCreateInfoKHR>().imageExtent;
            camera.set_aspect_ratio(extent.width, extent.height);
            for (size_t i = 0; i < wsiManager.get_command_buffers().size(); ++i) {
                const auto& commandBuffer = wsiManager.get_command_buffers()[i];
                vkResult = vkBeginCommandBuffer(commandBuffer, &gvk::get_default<VkCommandBufferBeginInfo>());
                assert(vkResult == VK_SUCCESS);
                auto renderPassBeginInfo = wsiManager.get_render_targets()[i].get_render_pass_begin_info();
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
                    for (auto& ball : balls) {
                        ball.record_cmds(commandBuffer, pipeline.get<gvk::PipelineLayout>());
                    }
                    for (auto& brick : bricks) {
                        brick.record_cmds(commandBuffer, pipeline.get<gvk::PipelineLayout>());
                    }
                }
                vkCmdEndRenderPass(commandBuffer);

                vkResult = vkEndCommandBuffer(commandBuffer);
                assert(vkResult == VK_SUCCESS);
            }
        }
        vkResult = dst_sample_acquire_submit_present(gfxContext);
        assert(vkResult == VK_SUCCESS);
    }
    vkResult = vkDeviceWaitIdle(gfxContext.get_devices()[0]);
    assert(vkResult == VK_SUCCESS);

    //remove the rigidbodies from the dynamics world and delete them
    for (int i = physicsContext.mupWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
        btCollisionObject* obj = physicsContext.mupWorld->getCollisionObjectArray()[i];
        // btRigidBody* body = btRigidBody::upcast(obj);
        physicsContext.mupWorld->removeCollisionObject(obj);
    }
    return 0;
}
