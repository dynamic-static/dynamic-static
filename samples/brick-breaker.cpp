
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

#include <map>
#include <unordered_set>
#include <utility>

VkResult create_pipeline(const gvk::RenderPass& renderPass, VkPolygonMode polygonMode, gvk::Pipeline* pPipeline)
{
    assert(renderPass);
    assert(pPipeline);

    // TODO : Documentation
    gvk::spirv::ShaderInfo vertexShaderInfo {
        .language = gvk::spirv::ShadingLanguage::Glsl,
        .stage = VK_SHADER_STAGE_VERTEX_BIT,
        .lineOffset = __LINE__,
        .source = R"(
            #version 450

            layout(set = 0, binding = 0)
            uniform CameraUniforms
            {
                mat4 view;
                mat4 projection;
            } camera;

            layout(set = 1, binding = 0)
            uniform ObjectUniforms
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
            uniform ObjectUniforms
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
    return dst_sample_create_pipeline<glm::vec3>(renderPass, VK_CULL_MODE_BACK_BIT, polygonMode, vertexShaderInfo, fragmentShaderInfo, pPipeline);
}

struct CameraUniforms
{
    glm::mat4 view { };
    glm::mat4 projection { };
};

struct ObjectUniforms
{
    glm::mat4 world { };
    glm::vec4 color { };
};

class GameObject final
{
public:
    struct BoxCreateInfo
    {
        btVector3 extents { 1, 1, 1 };
    };

    struct SphereCreateInfo
    {
        btScalar radius { 1 };
    };

    struct CreateInfo
    {
        const BoxCreateInfo* pBoxCreateInfo { nullptr };
        const SphereCreateInfo* pSphereCreateInfo { nullptr };
        dst::physics::RigidBody::CreateInfo rigidBodyCreateInfo { };
    };

    class Factory final
    {
    public:
        Factory(const gvk::DescriptorSetLayout& descriptorSetLayout, uint32_t objectCount)
            : mDescriptorSetLayout { descriptorSetLayout }
        {
            assert(mDescriptorSetLayout);
            std::vector<VkDescriptorPoolSize> descriptorPoolSizes;
            auto descriptorSetLayoutCreateInfo = mDescriptorSetLayout.get<VkDescriptorSetLayoutCreateInfo>();
            for (uint32_t binding_i = 0; binding_i < descriptorSetLayoutCreateInfo.bindingCount; ++binding_i) {
                const auto& binding = descriptorSetLayoutCreateInfo.pBindings[binding_i];
                descriptorPoolSizes.push_back({ binding.descriptorType, binding.descriptorCount * objectCount });
            }
            auto descriptorPoolCreateInfo = gvk::get_default<VkDescriptorPoolCreateInfo>();
            descriptorPoolCreateInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
            descriptorPoolCreateInfo.maxSets = objectCount;
            descriptorPoolCreateInfo.poolSizeCount = (uint32_t)descriptorPoolSizes.size();
            descriptorPoolCreateInfo.pPoolSizes = !descriptorPoolSizes.empty() ? descriptorPoolSizes.data() : nullptr;
            auto vkResult = gvk::DescriptorPool::create(mDescriptorSetLayout.get<gvk::Device>(), &descriptorPoolCreateInfo, nullptr, &mDescriptorPool);
            assert(vkResult == VK_SUCCESS);
            (void)vkResult;
        }

        void create_game_object(const gvk::CommandBuffer& commandBuffer, GameObject::CreateInfo createInfo, GameObject* pGameObject)
        {
            assert(commandBuffer);
            assert(!createInfo.pBoxCreateInfo != !createInfo.pSphereCreateInfo);
            assert(pGameObject);
            std::pair<btCollisionShape*, gvk::Mesh> resources;
            if (createInfo.pBoxCreateInfo) {
                resources = get_box_resources(commandBuffer, *createInfo.pBoxCreateInfo);
            } else {
                resources = get_sphere_resources(commandBuffer, *createInfo.pSphereCreateInfo);
            }
            pGameObject->mMesh = resources.second;
            createInfo.rigidBodyCreateInfo.pCollisionShape = resources.first;
            createInfo.rigidBodyCreateInfo.pUserData = this;
            dst::physics::RigidBody::create(&createInfo.rigidBodyCreateInfo, &pGameObject->rigidBody);
            create_descriptor_resources(pGameObject);
        }

    private:
        std::pair<btCollisionShape*, gvk::Mesh> get_box_resources(const gvk::CommandBuffer& commandBuffer, const GameObject::BoxCreateInfo& boxCreateInfo)
        {
            auto itr = mBoxResources.find(boxCreateInfo.extents);
            if (itr == mBoxResources.end()) {
                gvk::Mesh mesh;
                auto vkResult = dst_sample_create_box_mesh(commandBuffer, { boxCreateInfo.extents.x(), boxCreateInfo.extents.y(), boxCreateInfo.extents.z() }, &mesh);
                assert(vkResult == VK_SUCCESS);
                (void)vkResult;
                itr = mBoxResources.insert({ boxCreateInfo.extents, { btBoxShape(boxCreateInfo.extents * 0.5f), mesh } }).first;
            }
            return { &itr->second.first, itr->second.second };
        }

        std::pair<btCollisionShape*, gvk::Mesh> get_sphere_resources(const gvk::CommandBuffer& commandBuffer, const GameObject::SphereCreateInfo& sphereCreateInfo)
        {
            auto itr = mSphereResources.find(sphereCreateInfo.radius);
            if (itr == mSphereResources.end()) {
                gvk::Mesh mesh;
                auto vkResult = dst_sample_create_sphere_mesh(commandBuffer, sphereCreateInfo.radius, 1, &mesh);
                assert(vkResult == VK_SUCCESS);
                (void)vkResult;
                itr = mSphereResources.insert({ sphereCreateInfo.radius, { btSphereShape(sphereCreateInfo.radius), mesh } }).first;
            }
            return { &itr->second.first, itr->second.second };
        }

        void create_descriptor_resources(GameObject* pGameObject)
        {
            assert(pGameObject);

            // TODO : Documentation
            auto vkResult = dst_sample_create_uniform_buffer<ObjectUniforms>(mDescriptorSetLayout.get<gvk::Device>(), &pGameObject->mUniformBuffer);
            assert(vkResult == VK_SUCCESS);
            (void)vkResult;

            // TODO : Documentation
            auto descriptorSetAllocateInfo = gvk::get_default<VkDescriptorSetAllocateInfo>();
            descriptorSetAllocateInfo.descriptorPool = mDescriptorPool;
            descriptorSetAllocateInfo.descriptorSetCount = 1;
            descriptorSetAllocateInfo.pSetLayouts = &mDescriptorSetLayout.get<const VkDescriptorSetLayout&>();
            vkResult = gvk::DescriptorSet::allocate(mDescriptorPool.get<gvk::Device>(), &descriptorSetAllocateInfo, &pGameObject->mDescriptorSet);
            assert(vkResult == VK_SUCCESS);
            (void)vkResult;

            // TODO : Documentation
            auto descriptorBufferInfo = gvk::get_default<VkDescriptorBufferInfo>();
            descriptorBufferInfo.buffer = pGameObject->mUniformBuffer;
            auto writeDescriptorSet = gvk::get_default<VkWriteDescriptorSet>();
            writeDescriptorSet.descriptorCount = 1;
            writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            writeDescriptorSet.dstSet = pGameObject->mDescriptorSet;
            writeDescriptorSet.pBufferInfo = &descriptorBufferInfo;
            vkUpdateDescriptorSets(mDescriptorPool.get<gvk::Device>(), 1, &writeDescriptorSet, 0, nullptr);
        }

        gvk::DescriptorPool mDescriptorPool;
        gvk::DescriptorSetLayout mDescriptorSetLayout;
        std::map<btVector3, std::pair<btBoxShape, gvk::Mesh>> mBoxResources;
        std::map<btScalar, std::pair<btSphereShape, gvk::Mesh>> mSphereResources;
    };

    GameObject() = default;

    inline GameObject(GameObject && other) noexcept
    {
        *this = std::move(other);
    }

    inline GameObject& operator=(GameObject && other) noexcept
    {
        if (this != &other) {
            rigidBody = std::move(other.rigidBody);
            color = std::move(other.color);
            mMesh = std::move(other.mMesh);
            mUniformBuffer = std::move(other.mUniformBuffer);
            mDescriptorSet = std::move(other.mDescriptorSet);
            rigidBody.set_user_data(this);
        }
        return *this;
    }

    void update_uniform_buffer(const gvk::Device& device)
    {
        ObjectUniforms ubo { };
        btTransform transform { };
        if (rigidBody.get_state() != dst::physics::RigidBody::State::Dynamic) {
            rigidBody.mupMotionState->setWorldTransform(rigidBody.mupRigidBody->getCenterOfMassTransform());
        }
        rigidBody.mupMotionState->getWorldTransform(transform);
        transform.getOpenGLMatrix(&ubo.world[0][0]);
        ubo.color = color;
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

    dst::physics::RigidBody rigidBody;
    glm::vec4 color { gvk::math::Color::White };

private:
    gvk::Mesh mMesh;
    gvk::Buffer mUniformBuffer;
    gvk::DescriptorSet mDescriptorSet;
};

enum class State
{
    Play,
    Celebration,
    GameOver,
    Resetting,
};

struct ResetState
{
    btQuaternion rotation { };
    btVector3 translation { };
};

int main(int, const char* [])
{
    std::cout <<                                                                                       std::endl;
    std::cout << "================================================================================" << std::endl;
    std::cout << "    dynamic-static - Brick Breaker                                              " << std::endl;
    std::cout << "--------------------------------------------------------------------------------" << std::endl;
    std::cout << "                                                                                " << std::endl;
    std::cout << "    Break out all the bricks to win!                                            " << std::endl;
    std::cout << "                                                                                " << std::endl;
    std::cout << "    Controls                                                                    " << std::endl;
    std::cout << "        [Space] : Fire a ball, multiple balls may be in play                    " << std::endl;
    std::cout << "        [<][>]  : Move the paddle                                               " << std::endl;
    std::cout << "        [Esc]   : Quit                                                          " << std::endl;
    std::cout << "                                                                                " << std::endl;
    std::cout << "    Debug                                                                       " << std::endl;
    std::cout << "        [~]          : Toggle wireframe                                         " << std::endl;
    std::cout << "        [Q][W][E]                                                               " << std::endl;
    std::cout << "        [A][S][D]    : Move camera                                              " << std::endl;
    std::cout << "        [Left Shift] : Hold to enable camera move speed multiplier              " << std::endl;
    std::cout << "        [Left Mouse] : Hold to enable mouse look                                " << std::endl;
    std::cout << "                                                                                " << std::endl;
    std::cout << "================================================================================" << std::endl;
    std::cout <<                                                                                       std::endl;

    GfxContext gfxContext;
    auto vkResult = GfxContext::create("dynamic-static - Brick Breaker", &gfxContext);
    assert(vkResult == VK_SUCCESS);
    auto gvkDevice = gfxContext.get_devices()[0];
    auto gvkQueue = gvk::get_queue_family(gvkDevice, 0).queues[0];

    auto systemSurfaceCreateInfo = gvk::get_default<gvk::system::Surface::CreateInfo>();
    systemSurfaceCreateInfo.pTitle = gfxContext.get_instance().get<VkInstanceCreateInfo>().pApplicationInfo->pApplicationName;
    systemSurfaceCreateInfo.extent = { 1280, 720 };
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

    dst::physics::World::CreateInfo physicsWorldCreateInfo { };
    dst::physics::World physicsWorld;
    dst::physics::World::create(&physicsWorldCreateInfo, &physicsWorld);

    ///////////////////////////////////////////////////////////////////////////////
    dst::physics::Collider::Pool colliderPool;
    ///////////////////////////////////////////////////////////////////////////////

    gvk::Pipeline polygonPipeline;
    vkResult = create_pipeline(wsiManager.get_render_pass(), VK_POLYGON_MODE_FILL, &polygonPipeline);
    assert(vkResult == VK_SUCCESS);
    gvk::Pipeline wireframePipeline;
    vkResult = create_pipeline(wsiManager.get_render_pass(), VK_POLYGON_MODE_LINE, &wireframePipeline);
    assert(vkResult == VK_SUCCESS);
    auto pipeline = polygonPipeline;

    // TODO : Documentation
    auto descriptorPoolSize = gvk::get_default<VkDescriptorPoolSize>();
    descriptorPoolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    descriptorPoolSize.descriptorCount = 1; // BallCount + BrickCount + 1 paddle + 1 ceiling + 2 walls + 1 camera + 1 floor
    auto descriptorPoolCreateInfo = gvk::get_default<VkDescriptorPoolCreateInfo>();
    descriptorPoolCreateInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    descriptorPoolCreateInfo.maxSets = descriptorPoolSize.descriptorCount;
    descriptorPoolCreateInfo.poolSizeCount = 1;
    descriptorPoolCreateInfo.pPoolSizes = &descriptorPoolSize;
    gvk::DescriptorPool descriptorPool;
    vkResult = gvk::DescriptorPool::create(gfxContext.get_devices()[0], &descriptorPoolCreateInfo, nullptr, &descriptorPool);
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
    gvk::math::Camera camera;
    gvk::math::FreeCameraController cameraController;
    cameraController.moveSpeed = 12.4f;
    cameraController.set_camera(&camera);
    camera.nearPlane = 1.0f;
    camera.transform.translation.z = -64;
    gvk::Buffer cameraUniformBuffer;
    vkResult = dst_sample_create_uniform_buffer<CameraUniforms>(gvkDevice, &cameraUniformBuffer);
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

    GameObject::Factory gameObjectFactory(descriptorSetLayouts[1], 1000); // BallCount + BrickCount + 1 paddle + 3 walls

    // TODO : Documentation
    constexpr btScalar PlayFieldWidth        = 32;
    constexpr btScalar PlayFieldHeight       = 64;
    constexpr btScalar BarrierThickness      = 1;
    constexpr uint32_t PlayFieldBarrierCount = 3;
    const std::array<btVector3, PlayFieldBarrierCount> PlayFieldBarrierExtents {
        btVector3(PlayFieldWidth,   BarrierThickness, BarrierThickness), // Top
        btVector3(BarrierThickness, PlayFieldHeight,  BarrierThickness), // Left
        btVector3(BarrierThickness, PlayFieldHeight,  BarrierThickness), // Right
    };
    const std::array<btVector3, PlayFieldBarrierCount> PlayFieldBarrierPositions {
        btVector3(                  0,    PlayFieldHeight * 0.5f, 0), // Top
        btVector3( PlayFieldWidth * 0.5f,                   0,    0), // Left
        btVector3(-PlayFieldWidth * 0.5f,                   0,    0), // Right
    };
    constexpr btScalar PlayFieldBarrierRestitution = 0.6f;
    std::array<GameObject, PlayFieldBarrierCount> playFieldBarriers;
    for (size_t i = 0; i < playFieldBarriers.size(); ++i) {
        GameObject::BoxCreateInfo gameObjectBoxCreateInfo { };
        gameObjectBoxCreateInfo.extents = PlayFieldBarrierExtents[i];
        GameObject::CreateInfo gameObjectCreateInfo { };
        gameObjectCreateInfo.pBoxCreateInfo = &gameObjectBoxCreateInfo;
        gameObjectCreateInfo.rigidBodyCreateInfo.material.restitution = PlayFieldBarrierRestitution;
        gameObjectCreateInfo.rigidBodyCreateInfo.initialTransform.setOrigin(PlayFieldBarrierPositions[i]);
        gameObjectFactory.create_game_object(gfxContext.get_command_buffers()[0], gameObjectCreateInfo, &playFieldBarriers[i]);
        physicsWorld.make_static(playFieldBarriers[i].rigidBody);
    }

    // TODO : Documentation
    constexpr btScalar ContainerWidth        = PlayFieldWidth + PlayFieldWidth * 0.5f;
    constexpr btScalar ContainerHeight       = PlayFieldHeight + PlayFieldHeight * 0.18f;
    constexpr btScalar ContainerDepth        = 32;
    constexpr uint32_t ContainerBarrierCount = 5;
    const std::array<btVector3, ContainerBarrierCount> ContainerBarrierExtents {
        btVector3(ContainerWidth,   ContainerHeight,  BarrierThickness), // Back
        btVector3(BarrierThickness, ContainerHeight,  ContainerDepth),   // Left
        btVector3(BarrierThickness, ContainerHeight,  ContainerDepth),   // Right
        btVector3(ContainerWidth,   ContainerHeight,  BarrierThickness), // Front
        btVector3(ContainerWidth,   BarrierThickness, ContainerDepth),   // Bottom
    };
    const std::array<btVector3, ContainerBarrierCount> ContainerBarrierPositions {
        btVector3(                  0,                       0,     ContainerDepth * 0.5f), // Back
        btVector3( ContainerWidth * 0.5f,                    0,                      0),    // Left
        btVector3(-ContainerWidth * 0.5f,                    0,                      0),    // Right
        btVector3(                  0,                       0,    -ContainerDepth * 0.5f), // Front
        btVector3(                  0,    -ContainerHeight * 0.5f,                   0),    // Bottom
    };
    std::array<GameObject, ContainerBarrierCount> containerBarriers;
    for (size_t i = 0; i < containerBarriers.size(); ++i) {
        GameObject::BoxCreateInfo gameObjectBoxCreateInfo { };
        gameObjectBoxCreateInfo.extents = ContainerBarrierExtents[i];
        GameObject::CreateInfo gameObjectCreateInfo { };
        gameObjectCreateInfo.pBoxCreateInfo = &gameObjectBoxCreateInfo;
        gameObjectCreateInfo.rigidBodyCreateInfo.initialTransform.setOrigin(ContainerBarrierPositions[i]);
        gameObjectFactory.create_game_object(gfxContext.get_command_buffers()[0], gameObjectCreateInfo, &containerBarriers[i]);
        physicsWorld.make_static(containerBarriers[i].rigidBody);
    }

    // TODO : Documentation
    constexpr btScalar BrickMass       = 8;
    constexpr btScalar BrickWidth      = 2;
    constexpr btScalar BrickHeight     = 1;
    constexpr btScalar BrickDepth      = 1;
    constexpr uint32_t BrickRowCount   = 6;
    constexpr uint32_t BrickColumCount = 10;
    constexpr uint32_t BrickCount      = BrickRowCount * BrickColumCount;
    const std::array<glm::vec4, BrickRowCount> BrickRowColors {
        gvk::math::Color::Red,
        gvk::math::Color::Orange,
        gvk::math::Color::Yellow,
        gvk::math::Color::Green,
        gvk::math::Color::DodgerBlue,
        gvk::math::Color::Violet,
    };
    const std::array<btVector3, BrickCount> BrickPositions {
        [&]()
        {
            const btScalar PlayAreaWidth = PlayFieldWidth - BarrierThickness;
            const btScalar BrickAreaWidth = PlayAreaWidth / BrickColumCount;
            std::array<btVector3, BrickCount> brickPositions { };
            for (uint32_t row_i = 0; row_i < BrickRowCount; ++row_i) {
                auto x = -PlayAreaWidth * 0.5f + BrickAreaWidth * 0.5f;
                for (uint32_t brick_i = 0; brick_i < BrickColumCount; ++brick_i) {
                    auto y = 30.0f - row_i * BrickHeight * 2.0f;
                    brickPositions[row_i * BrickColumCount + brick_i] = { x, y, 0 };
                    x += BrickAreaWidth;
                }
            }
            return brickPositions;
        }()
    };
    std::array<GameObject, BrickCount> bricks;
    std::unordered_set<GameObject*> liveBricks;
    for (uint32_t i = 0; i < BrickCount; ++i) {
        GameObject::BoxCreateInfo gameObjectBoxCreateInfo { };
        gameObjectBoxCreateInfo.extents = { BrickWidth, BrickHeight, BrickDepth };
        GameObject::CreateInfo gameObjectCreateInfo { };
        gameObjectCreateInfo.pBoxCreateInfo = &gameObjectBoxCreateInfo;
        gameObjectCreateInfo.rigidBodyCreateInfo.mass = BrickMass;
        gameObjectCreateInfo.rigidBodyCreateInfo.initialTransform.setOrigin(BrickPositions[i]);
        gameObjectFactory.create_game_object(gfxContext.get_command_buffers()[0], gameObjectCreateInfo, &bricks[i]);
        bricks[i].color = BrickRowColors[i / BrickColumCount];
        physicsWorld.make_static(bricks[i].rigidBody);
        liveBricks.insert(&bricks[i]);

        initialPositions.insert({ (uint64_t)bricks[i].rigidBody.mupRigidBody.get(), BrickPositions[i] });
    }

    // TODO : Documentation
    constexpr btScalar BallRadius = 0.5f;
    constexpr btScalar BallMass   = 1;
    constexpr uint32_t BallCount  = 3;
    const std::array<btVector3, BallCount> BallPositions {
        [&]()
        {
            const btScalar Gap = BallRadius * 4;
            std::array<btVector3, BallCount> ballPositions { };
            for (uint32_t i = 0; i < BallCount; ++i) {
                auto x = -PlayFieldWidth * 0.5f + i * Gap;
                auto y = PlayFieldHeight * 0.5f + Gap;
                ballPositions[i] = { x, y, 0 };
            }
            return ballPositions;
        }()
    };
    constexpr btScalar BallRestitution = 0.9f;
    std::array<GameObject, BallCount> balls;
    for (uint32_t i = 0; i < BallCount; ++i) {
        GameObject::SphereCreateInfo gameObjectSphereCreateInfo { };
        gameObjectSphereCreateInfo.radius = BallRadius;
        GameObject::CreateInfo gameObjectCreateInfo { };
        gameObjectCreateInfo.pSphereCreateInfo = &gameObjectSphereCreateInfo;
        gameObjectCreateInfo.rigidBodyCreateInfo.mass = BallMass;
        gameObjectCreateInfo.rigidBodyCreateInfo.material.restitution = BallRestitution;
        gameObjectCreateInfo.rigidBodyCreateInfo.linearFactor = { 1, 1, 0 };
        gameObjectCreateInfo.rigidBodyCreateInfo.initialTransform.setOrigin(BallPositions[i]);
        gameObjectFactory.create_game_object(gfxContext.get_command_buffers()[0], gameObjectCreateInfo, &balls[i]);
        balls[i].color = gvk::math::Color::SlateGray;

        initialPositions.insert({ (uint64_t)balls[i].rigidBody.mupRigidBody.get(), BallPositions[i] });
    }

    // TODO : Documentation
    constexpr btScalar PaddleWidth  = 6;
    constexpr btScalar PaddleHeight = 1;
    constexpr btScalar PaddleDepth  = 0.1f;
    constexpr btScalar PaddleMass   = 1;
    GameObject paddle;
    {
        GameObject::BoxCreateInfo gameObjectBoxCreateInfo { };
        gameObjectBoxCreateInfo.extents = { PaddleWidth, PaddleHeight, PaddleDepth };
        GameObject::CreateInfo gameObjectCreateInfo { };
        gameObjectCreateInfo.rigidBodyCreateInfo.mass = PaddleMass;
        gameObjectCreateInfo.rigidBodyCreateInfo.linearDamping = 0.4f;
        gameObjectCreateInfo.rigidBodyCreateInfo.linearFactor = { 1, 0, 0 };
        gameObjectCreateInfo.rigidBodyCreateInfo.angularFactor = { 0, 0, 0 };
        gameObjectCreateInfo.rigidBodyCreateInfo.initialTransform.setOrigin({ 0, -PlayFieldHeight * 0.5f + PaddleHeight * 4, 0 });
        gameObjectCreateInfo.pBoxCreateInfo = &gameObjectBoxCreateInfo;
        gameObjectFactory.create_game_object(gfxContext.get_command_buffers()[0], gameObjectCreateInfo, &paddle);
        paddle.color = gvk::math::Color::Brown;
        physicsWorld.make_dynamic(paddle.rigidBody);
    }

    // TODO : Documentation
    float celebrationTimer = 0;

    // TODO : Documentation
    float resetTimer = 0;
    std::map<uint64_t, ResetState> resetStates;

    uint32_t ballCount = BallCount;
    State state = State::Play;
    // double frameTimeAccumulator = 0;
    gvk::system::Clock clock;
    while (
        !(systemSurface.get_input().keyboard.down(gvk::system::Key::Escape)) &&
        !(systemSurface.get_status() & gvk::system::Surface::CloseRequested)) {

        // TODO : Documentation
        clock.update();
        auto deltaTime = clock.elapsed<gvk::system::Seconds<float>>();

        // TODO : Documentation
        gvk::system::Surface::update();
        const auto& input = systemSurface.get_input();

        // TODO : Documentation
        if (input.keyboard.pressed(gvk::system::Key::OEM_Tilde)) {
            if (pipeline == polygonPipeline) {
                pipeline = wireframePipeline;
            } else {
                pipeline = polygonPipeline;
            }
        }

        // TODO : Documentation
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

        // TODO : Documentation
        constexpr btScalar PaddleForce = 48;
        if (input.keyboard.down(gvk::system::Key::LeftArrow)) {
            paddle.rigidBody.apply_force({ PaddleForce, 0, 0 });
        }
        if (input.keyboard.down(gvk::system::Key::RightArrow)) {
            paddle.rigidBody.apply_force({ -PaddleForce, 0, 0 });
        }

        // TODO : Documentation
        switch (state) {
        case State::Play: {
            // TODO : Documentation
            if (input.keyboard.pressed(gvk::system::Key::SpaceBar)) {
                for (auto ritr = balls.rbegin(); ritr != balls.rend(); ++ritr) {
                    auto& rigidBody = ritr->rigidBody;
                    if (rigidBody.get_state() == dst::physics::RigidBody::State::Disabled) {
                        rigidBody.halt();
                        rigidBody.set_transform(btTransform::getIdentity());
                        physicsWorld.make_dynamic(rigidBody);
                        break;
                    }
                }

#if 0
                if (ballCount) {
                    assert(ballCount <= balls.size());
                    auto& ball = balls[ballCount - 1];
                    ball.rigidBody.halt();
                    ball.rigidBody.set_transform(btTransform::getIdentity());
                    physicsWorld.make_dynamic(ball.rigidBody);
                    ballCount -= 1;
                }
#endif
            }

            // TODO : Documentation
            for (auto itr = liveBricks.begin(); itr != liveBricks.end();) {
                auto& rigidBody = (*itr)->rigidBody;
                if (physicsWorld.get_collided_rigid_bodies().count(&rigidBody)) {
                    physicsWorld.disable(rigidBody);
                    physicsWorld.make_dynamic(rigidBody);
                    liveBricks.erase(itr++);
                } else {
                    ++itr;
                }
            }

            // TODO : Documentation
            uint32_t liveBallCount = 0;
            const btScalar OutOfPlay = -PlayFieldHeight * 0.5f;
            const auto& paddlePosition = paddle.rigidBody.get_transform().getOrigin();
            for (auto& ball : balls) {
                const auto& ballPosition = ball.rigidBody.get_transform().getOrigin();
                if (OutOfPlay < ballPosition.y()) {
                    ++liveBallCount;
                    if (physicsWorld.get_collisions().count(dst::physics::make_collision(&ball.rigidBody, &paddle.rigidBody))) {
                        if (paddlePosition.y() < ballPosition.y()) {
                            auto impulse = (ballPosition - paddlePosition).normalized();
                            impulse *= 64;
                            impulse.setY(64);
                            ball.rigidBody.apply_impulse(impulse);
                        }
                    }
                }
            }

            // TODO : Documentation
            if (liveBricks.empty()) {
                celebrationTimer = 0;
                state = State::Celebration;
            } else if (!liveBallCount) {
                state = State::GameOver;
            }
        } break;
        case State::Celebration: {
            // TODO : Documentation
            constexpr float CelebrationDuration = 4.5f;
            constexpr float CelebrationColorDuration = 0.01f;
            std::vector<glm::vec4> celebrationColors {
                gvk::math::Color::Red,
                gvk::math::Color::White,
                gvk::math::Color::Blue,
                gvk::math::Color::Yellow
            };
            celebrationTimer += clock.elapsed<gvk::system::Seconds<float>>();
            if (celebrationTimer < CelebrationDuration) {
                auto index = (size_t)std::round(celebrationTimer / CelebrationColorDuration) % celebrationColors.size();
                for (auto& wall : playFieldBarriers) {
                    wall.color = celebrationColors[index];
                }
            } else {
                state = State::GameOver;
                for (auto& wall : playFieldBarriers) {
                    wall.color = gvk::math::Color::White;
                }
            }
        } break;
        case State::GameOver: {
            // TODO : Documentation
            resetTimer = 0;
            resetStates.clear();
            state = State::Resetting;
            for (auto& brick : bricks) {
                liveBricks.insert(&brick);
                brick.rigidBody.mupRigidBody->setLinearVelocity(btVector3(0, 0, 0));
                brick.rigidBody.mupRigidBody->setAngularVelocity(btVector3(0, 0, 0));

                physicsWorld.disable(brick.rigidBody);
                auto transform = brick.rigidBody.mupRigidBody->getCenterOfMassTransform();
                ResetState resetState { };
                resetState.rotation = transform.getRotation();
                resetState.translation = transform.getOrigin();
                resetStates.insert({ (uint64_t)brick.rigidBody.mupRigidBody.get(), resetState });
            }
            for (auto& ball : balls) {
                // liveBalls.insert(&ball);
                ball.rigidBody.mupRigidBody->setLinearVelocity(btVector3(0, 0, 0));
                ball.rigidBody.mupRigidBody->setAngularVelocity(btVector3(0, 0, 0));

                physicsWorld.disable(ball.rigidBody);
                auto transform = ball.rigidBody.mupRigidBody->getCenterOfMassTransform();
                ResetState resetState { };
                resetState.rotation = transform.getRotation();
                resetState.translation = transform.getOrigin();
                resetStates.insert({ (uint64_t)ball.rigidBody.mupRigidBody.get(), resetState });
            }
        } break;
        case State::Resetting: {
            // TODO : Documentation
            constexpr float ResetDuration = 2.5f;
            resetTimer += clock.elapsed<gvk::system::Seconds<float>>();
            if (resetTimer < ResetDuration) {
                float t = resetTimer / ResetDuration;
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
                for (auto& brick : bricks) {
                    liveBricks.insert(&brick);
                    brick.rigidBody.mupMotionState->setWorldTransform(brick.rigidBody.mupRigidBody->getCenterOfMassTransform());
                    physicsWorld.make_static(brick.rigidBody);
                }
                for (auto& ball : balls) {
                    ball.rigidBody.mupMotionState->setWorldTransform(ball.rigidBody.mupRigidBody->getCenterOfMassTransform());
                    // liveBalls.insert(&ball);
                }
                resetStates.clear();
                ballCount = 3;
                state = State::Play;
            }
        } break;
        }

        // TODO : Documentation
        auto paddleTransform = paddle.rigidBody.get_transform();
        auto paddleX = paddleTransform.getOrigin().x();
        auto xMax = PlayFieldWidth * 0.5f - PaddleWidth * 0.5f;
        auto xMin = -xMax;
        paddleTransform.getOrigin().setX(glm::clamp(paddleX, xMin, xMax));
        paddle.rigidBody.set_transform(paddleTransform);

        // TODO : Documentation
        physicsWorld.update(deltaTime);

        // TODO : Documentation
        CameraUniforms cameraUbo { };
        cameraUbo.view = camera.view();
        cameraUbo.projection = camera.projection();
        VmaAllocationInfo allocationInfo { };
        vmaGetAllocationInfo(gfxContext.get_devices()[0].get<VmaAllocator>(), cameraUniformBuffer.get<VmaAllocation>(), &allocationInfo);
        assert(allocationInfo.pMappedData);
        memcpy(allocationInfo.pMappedData, &cameraUbo, sizeof(CameraUniforms));

        // TODO : Documentation
        paddle.update_uniform_buffer(gfxContext.get_devices()[0]);
        for (auto& wall : playFieldBarriers) {
            wall.update_uniform_buffer(gfxContext.get_devices()[0]);
        }
        for (auto& brick : bricks) {
            brick.update_uniform_buffer(gfxContext.get_devices()[0]);
        }
        for (auto& ball : balls) {
            ball.update_uniform_buffer(gfxContext.get_devices()[0]);
        }

        // TODO : Documentation
        if (pipeline == wireframePipeline) {
            for (auto& backStop : containerBarriers) {
                backStop.update_uniform_buffer(gfxContext.get_devices()[0]);
            }
        }

        // TODO : Documentation
        wsiManager.update();
        auto swapchain = wsiManager.get_swapchain();
        if (swapchain) {
            // TODO : Documentation
            uint32_t imageIndex = 0;
            vkResult = wsiManager.acquire_next_image(UINT64_MAX, VK_NULL_HANDLE, &imageIndex);
            assert(vkResult == VK_SUCCESS || vkResult == VK_SUBOPTIMAL_KHR);

            // TODO : Documentation
            auto extent = wsiManager.get_swapchain().get<VkSwapchainCreateInfoKHR>().imageExtent;
            camera.set_aspect_ratio(extent.width, extent.height);

            // TODO : Documentation
            const auto& vkFences = wsiManager.get_vk_fences();
            vkResult = vkWaitForFences(gvkDevice, 1, &vkFences[imageIndex], VK_TRUE, UINT64_MAX);
            assert(vkResult == VK_SUCCESS);
            vkResult = vkResetFences(gvkDevice, 1, &vkFences[imageIndex]);
            assert(vkResult == VK_SUCCESS);

            // TODO : Documentation
            const auto& commandBuffer = wsiManager.get_command_buffers()[imageIndex];
            vkResult = vkBeginCommandBuffer(commandBuffer, &gvk::get_default<VkCommandBufferBeginInfo>());
            assert(vkResult == VK_SUCCESS);
            auto renderPassBeginInfo = wsiManager.get_render_targets()[imageIndex].get_render_pass_begin_info();
            vkCmdBeginRenderPass(commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

            // TODO : Documentation
            VkRect2D scissor { .extent = renderPassBeginInfo.renderArea.extent };
            vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
            VkViewport viewport { .width = (float)scissor.extent.width, .height = (float)scissor.extent.height, .minDepth = 0, .maxDepth = 1 };
            vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

            // TODO : Documentation
            vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

            // TODO : Documentation
            const auto& pipelineLayout = pipeline.get<gvk::PipelineLayout>();
            vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &(const VkDescriptorSet&)cameraDescriptorSet, 0, nullptr);

            // TODO : Documentation
            paddle.record_cmds(commandBuffer, pipelineLayout);
            for (auto& wall : playFieldBarriers) {
                wall.record_cmds(commandBuffer, pipelineLayout);
            }
            for (auto& brick : bricks) {
                brick.record_cmds(commandBuffer, pipelineLayout);
            }
            for (auto& ball : balls) {
                ball.record_cmds(commandBuffer, pipelineLayout);
            }

            // TODO : Documentation
            if (pipeline == wireframePipeline) {
                for (auto& backStop : containerBarriers) {
                    backStop.record_cmds(commandBuffer, pipelineLayout);
                }
            }

            // TODO : Documentation
            vkCmdEndRenderPass(commandBuffer);
            vkResult = vkEndCommandBuffer(commandBuffer);
            assert(vkResult == VK_SUCCESS);

            // TODO : Documentation
            auto submitInfo = wsiManager.get_submit_info(imageIndex);
            vkResult = vkQueueSubmit(gvkQueue, 1, &submitInfo, vkFences[imageIndex]);
            assert(vkResult == VK_SUCCESS);

            // TODO : Documentation
            auto presentInfo = wsiManager.get_present_info(&imageIndex);
            vkResult = vkQueuePresentKHR(gvkQueue, &presentInfo);
            assert(vkResult == VK_SUCCESS || vkResult == VK_SUBOPTIMAL_KHR);
        }
    }

    // TODO : Documentation
    vkResult = vkDeviceWaitIdle(gfxContext.get_devices()[0]);
    assert(vkResult == VK_SUCCESS);

    // TODO : Documentation
    physicsWorld.reset();

    return 0;
}
