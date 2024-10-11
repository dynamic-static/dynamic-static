
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
#include <tuple>
#include <unordered_set>
#include <utility>

VkResult create_pipeline(const gvk::RenderPass& renderPass, VkPolygonMode polygonMode, gvk::Pipeline* pPipeline)
{
    assert(renderPass);
    assert(pPipeline);

    // A simple vertex shader (GPU program that processes individual vertices) that
    //  draws a mesh comprised of vec3 vertices.  This shader's interface is made
    //  up of two uniform buffers, CameraUniforms bound at DescriptorSet 0, and
    //  ObjectUniforms bound at DescriptorSet 1.
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

            layout(location = 0) in vec3 vertexPosition;

            out gl_PerVertex
            {
                vec4 gl_Position;
            };

            void main()
            {
                gl_Position = camera.projection * camera.view * object.world * vec4(vertexPosition, 1);
            }
        )"
    };

    // A simple fragment shader (GPU program that processes individual fragments)
    //  that writes the object color provided by the ObjectUniforms bound at
    //  DescriptorSet 1.
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

            layout(location = 0) out vec4 fragmentColor;

            void main()
            {
                fragmentColor = object.color;
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
        inline Factory(const gvk::DescriptorPool& descriptorPool, const gvk::DescriptorSetLayout& descriptorSetLayout)
            : mDescriptorPool { descriptorPool }
            , mDescriptorSetLayout { descriptorSetLayout }
        {
            assert(mDescriptorPool);
            assert(mDescriptorSetLayout);
        }

        inline void create_game_object(const gvk::CommandBuffer& commandBuffer, GameObject::CreateInfo createInfo, GameObject* pGameObject)
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
        inline std::pair<btCollisionShape*, gvk::Mesh> get_box_resources(const gvk::CommandBuffer& commandBuffer, const GameObject::BoxCreateInfo& boxCreateInfo)
        {
            // Check if a btCollisionShape and gvk::Mesh have already been created for a
            //  box with the given extents.  If so return the existing resources, otherwise
            //  create new reosurces.
            auto itr = mBoxResources.find(boxCreateInfo.extents);
            if (itr == mBoxResources.end()) {
                gvk::Mesh mesh;
                dst_vk_result(dst_sample_create_box_mesh(commandBuffer, { boxCreateInfo.extents.x(), boxCreateInfo.extents.y(), boxCreateInfo.extents.z() }, &mesh));
                itr = mBoxResources.insert({ boxCreateInfo.extents, { btBoxShape(boxCreateInfo.extents * 0.5f), mesh } }).first;
            }
            return { &itr->second.first, itr->second.second };
        }

        inline std::pair<btCollisionShape*, gvk::Mesh> get_sphere_resources(const gvk::CommandBuffer& commandBuffer, const GameObject::SphereCreateInfo& sphereCreateInfo)
        {
            // Check if a btCollisionShape and gvk::Mesh have already been created for a
            //  sphere with the given radius.  If so return the existing resources,
            //  otherwise create new reosurces.
            auto itr = mSphereResources.find(sphereCreateInfo.radius);
            if (itr == mSphereResources.end()) {
                gvk::Mesh mesh;
                dst_vk_result(dst_sample_create_sphere_mesh(commandBuffer, sphereCreateInfo.radius, 1, &mesh));
                itr = mSphereResources.insert({ sphereCreateInfo.radius, { btSphereShape(sphereCreateInfo.radius), mesh } }).first;
            }
            return { &itr->second.first, itr->second.second };
        }

        inline void create_descriptor_resources(GameObject* pGameObject)
        {
            assert(pGameObject);

            // Create a uniform Buffer.  This Buffer will be populated with data that is
            //  needed to process the GameObject during vertex/fragment shading.
            dst_vk_result(dst_sample_create_uniform_buffer<ObjectUniforms>(mDescriptorSetLayout.get<gvk::Device>(), &pGameObject->mUniformBuffer));

            // Allocate a DescriptorSet.  DescriptorSets are used to bind resources (like
            //  uniform Buffers) for consumption by the vertex/fragment shader.
            auto descriptorSetAllocateInfo = gvk::get_default<VkDescriptorSetAllocateInfo>();
            descriptorSetAllocateInfo.descriptorPool = mDescriptorPool;
            descriptorSetAllocateInfo.descriptorSetCount = 1;
            descriptorSetAllocateInfo.pSetLayouts = &mDescriptorSetLayout.get<VkDescriptorSetLayout>();
            dst_vk_result(gvk::DescriptorSet::allocate(mDescriptorPool.get<gvk::Device>(), &descriptorSetAllocateInfo, &pGameObject->mDescriptorSet));

            // Update the GameObject's DescriptorSet to reference the uniform Buffer.
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

    inline void update_uniform_buffer(const gvk::Device& device) const
    {
        // Write this GameObject's ObjectUniforms data into the uniform Buffer.
        ObjectUniforms ubo { };
        rigidBody.get_motion_state_transform().getOpenGLMatrix(&ubo.world[0][0]);
        ubo.color = color;
        VmaAllocationInfo allocationInfo { };
        vmaGetAllocationInfo(device.get<VmaAllocator>(), mUniformBuffer.get<VmaAllocation>(), &allocationInfo);
        assert(allocationInfo.pMappedData);
        memcpy(allocationInfo.pMappedData, &ubo, sizeof(ObjectUniforms));
    }

    inline void record_draw_cmds(const gvk::CommandBuffer& commandBuffer, const gvk::PipelineLayout& pipelineLayout) const
    {
        // Bind this GameObject's DescriptorSet (which references the uniform Buffer)
        //  then record the gvk::Mesh's draw cmds.
        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 1, 1, &mDescriptorSet.get<VkDescriptorSet>(), 0, nullptr);
        mMesh.record_cmds(commandBuffer);
    }

    dst::physics::RigidBody rigidBody;
    glm::vec4 color { gvk::math::Color::White };

private:
    gvk::Mesh mMesh;
    gvk::Buffer mUniformBuffer;
    gvk::DescriptorSet mDescriptorSet;
};

enum class GameState
{
    Playing,
    Celebration,
    GameOver,
    Resetting,
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

    // Create a gvk::Context.  This will initialize a VkInstance and VkDevice.
    gvk::Context gvkContext;
    dst_vk_result(dst_sample_create_gvk_context("dynamic-static - Brick Breaker", &gvkContext));
    auto gvkDevice = gvkContext.get<gvk::Devices>()[0];
    auto gvkQueue = gvk::get_queue_family(gvkDevice, 0).queues[0];

    // Create a gvk::system::Surface.  This is used to control a system window.
    auto systemSurfaceCreateInfo = gvk::get_default<gvk::system::Surface::CreateInfo>();
    systemSurfaceCreateInfo.pTitle = gvkContext.get<gvk::Instance>().get<VkInstanceCreateInfo>().pApplicationInfo->pApplicationName;
    systemSurfaceCreateInfo.extent = { 1280, 720 };
    gvk::system::Surface gvkSystemSurface = gvk::nullref;
    dst_vk_result((VkResult)gvk::system::Surface::create(&systemSurfaceCreateInfo, &gvkSystemSurface));

    // Create a gvk::SurfaceKHR
    auto win32SurfaceCreateInfo = gvk::get_default<VkWin32SurfaceCreateInfoKHR>();
    win32SurfaceCreateInfo.hinstance = GetModuleHandle(NULL);
    win32SurfaceCreateInfo.hwnd = gvkSystemSurface.get<gvk::system::Surface::PlatformInfo>().hwnd;
    gvk::SurfaceKHR gvkSurface = VK_NULL_HANDLE;
    dst_vk_result(gvk::SurfaceKHR::create(gvkContext.get<gvk::Instance>(), &win32SurfaceCreateInfo, nullptr, &gvkSurface));

    // Create a gvk::WsiManager.  This is used to manage a connection between the
    //  Vulkan context and the system window.
    auto wsiContextCreateInfo = gvk::get_default<gvk::wsi::Context::CreateInfo>();
    wsiContextCreateInfo.sampleCount = VK_SAMPLE_COUNT_64_BIT;
    wsiContextCreateInfo.depthFormat = VK_FORMAT_D32_SFLOAT;
    wsiContextCreateInfo.presentMode = VK_PRESENT_MODE_MAILBOX_KHR;
    wsiContextCreateInfo.queueFamilyIndex = gvkQueue.get<VkDeviceQueueCreateInfo>().queueFamilyIndex;
    gvk::wsi::Context wsiContext = gvk::nullref;
    dst_vk_result(gvk::wsi::Context::create(gvkDevice, gvkSurface, &wsiContextCreateInfo, nullptr, &wsiContext));

    // Create two Pipelines.  These are identical except for the VkPolygonMode.
    //  polygonPipeline is used normally.  The wireframePipeline can be toggled for
    //  debugging.
    gvk::Pipeline polygonPipeline;
    dst_vk_result(create_pipeline(wsiContext.get<gvk::RenderPass>(), VK_POLYGON_MODE_FILL, &polygonPipeline));
    gvk::Pipeline wireframePipeline;
    dst_vk_result(create_pipeline(wsiContext.get<gvk::RenderPass>(), VK_POLYGON_MODE_LINE, &wireframePipeline));
    auto pipeline = polygonPipeline;

    // Create a DescriptorPool.  This DescriptorPool provides 1000 uniform Buffer
    //  descriptors.  Only one is needed for each GameObject, but this leaves
    //  plenty of headroom to create more GameObjects without having to change the
    //  DescriptorPool.
    auto descriptorPoolSize = gvk::get_default<VkDescriptorPoolSize>();
    descriptorPoolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    descriptorPoolSize.descriptorCount = 1000;
    auto descriptorPoolCreateInfo = gvk::get_default<VkDescriptorPoolCreateInfo>();
    descriptorPoolCreateInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    descriptorPoolCreateInfo.maxSets = descriptorPoolSize.descriptorCount;
    descriptorPoolCreateInfo.poolSizeCount = 1;
    descriptorPoolCreateInfo.pPoolSizes = &descriptorPoolSize;
    gvk::DescriptorPool descriptorPool;
    dst_vk_result(gvk::DescriptorPool::create(gvkContext.get<gvk::Devices>()[0], &descriptorPoolCreateInfo, nullptr, &descriptorPool));

    // Get the DescriptorSetLayouts from the Pipeline.  There should be two...
    //      `layout(set = 0, binding = 0) uniform CameraUniforms`
    //          used in the vertex shader
    //  and
    //      `layout(set = 1, binding = 0) uniform ObjectUniforms`
    //          used in both the vertex and fragment shaders
    const auto& descriptorSetLayouts = pipeline.get<gvk::PipelineLayout>().get<gvk::DescriptorSetLayouts>();
    assert(descriptorSetLayouts.size() == 2);
    const auto& cameraDescriptorSetLayout = descriptorSetLayouts[0];
    const auto& objectDescriptorSetLayout = descriptorSetLayouts[1];

    // Create a GameObject::Factory.  GameObject::Factory initializes graphics and
    //  resources for GameObjects so provide it with the DescriptorPool and object
    //  DescriptorSetLayout so it can allocate uniform Buffer descriptors when
    //  creating GameObjects.
    GameObject::Factory gameObjectFactory(descriptorPool, objectDescriptorSetLayout);

    // Create a dst::physics::World.
    dst::physics::World::CreateInfo physicsWorldCreateInfo { };
    dst::physics::World physicsWorld;
    dst::physics::World::create(&physicsWorldCreateInfo, &physicsWorld);

    // Create a Camera.
    gvk::math::Camera camera;
    gvk::math::FreeCameraController cameraController;
    cameraController.moveSpeed = 12.4f;
    cameraController.set_camera(&camera);
    camera.nearPlane = 1.0f;
    camera.transform.translation.z = -64;

    // The Camera needs a uniform Buffer to interface with the vertex shader.
    gvk::Buffer cameraUniformBuffer;
    dst_vk_result(dst_sample_create_uniform_buffer<CameraUniforms>(gvkDevice, &cameraUniformBuffer));

    // Allocate a DescriptorSet for the Camera.
    auto descriptorSetAllocateInfo = gvk::get_default<VkDescriptorSetAllocateInfo>();
    descriptorSetAllocateInfo.descriptorPool = descriptorPool;
    descriptorSetAllocateInfo.descriptorSetCount = 1;
    descriptorSetAllocateInfo.pSetLayouts = &cameraDescriptorSetLayout.get<VkDescriptorSetLayout>();
    gvk::DescriptorSet cameraDescriptorSet;
    dst_vk_result(gvk::DescriptorSet::allocate(gvkContext.get<gvk::Devices>()[0], &descriptorSetAllocateInfo, &cameraDescriptorSet));

    // Update the Camera's DescriptorSet to reference the Camera's uniform Buffer.
    auto descriptorBufferInfo = gvk::get_default<VkDescriptorBufferInfo>();
    descriptorBufferInfo.buffer = cameraUniformBuffer;
    auto writeDescriptorSet = gvk::get_default<VkWriteDescriptorSet>();
    writeDescriptorSet.descriptorCount = 1;
    writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writeDescriptorSet.dstSet = cameraDescriptorSet;
    writeDescriptorSet.pBufferInfo = &descriptorBufferInfo;
    vkUpdateDescriptorSets(gvkContext.get<gvk::Devices>()[0], 1, &writeDescriptorSet, 0, nullptr);

    // Create the play field made up of the side and top barriers that the player
    //  sees.
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
        gameObjectFactory.create_game_object(gvkContext.get<gvk::CommandBuffers>()[0], gameObjectCreateInfo, &playFieldBarriers[i]);
        physicsWorld.make_static(playFieldBarriers[i].rigidBody);
    }

    // Create the container surrounding the play field.  This container provides a
    //  boundary for bricks/balls that have gone out of play.  The container isn't
    //  rendered unless the wireframe Pipeline is enabled.
    constexpr btScalar ContainerWidth        = PlayFieldWidth + PlayFieldWidth * 0.5f;
    constexpr btScalar ContainerHeight       = PlayFieldHeight + PlayFieldHeight * 0.18f;
    constexpr btScalar ContainerDepth        = 16;
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
        gameObjectFactory.create_game_object(gvkContext.get<gvk::CommandBuffers>()[0], gameObjectCreateInfo, &containerBarriers[i]);
        physicsWorld.make_static(containerBarriers[i].rigidBody);
    }

    // Create the bricks.
    constexpr btScalar BrickMass       = 8;
    constexpr btScalar BrickWidth      = 2;
    constexpr btScalar BrickHeight     = 1;
    constexpr btScalar BrickDepth      = 1;
    constexpr uint32_t BrickRowCount   = 6;
    constexpr uint32_t BrickColumCount = 10;
    constexpr uint32_t BrickCount      = BrickRowCount * BrickColumCount;
    const std::array<glm::vec4, BrickRowCount> BrickRowColors {
        gvk::math::Color::Red,
        gvk::math::Color::DarkOrange,
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
    std::array<GameObject, BrickCount> bricks { };
    std::unordered_set<GameObject*> liveBricks;
    std::array<std::tuple<btVector3, btQuaternion>, BrickCount> deadBricks { };
    for (uint32_t i = 0; i < BrickCount; ++i) {
        GameObject::BoxCreateInfo gameObjectBoxCreateInfo { };
        gameObjectBoxCreateInfo.extents = { BrickWidth, BrickHeight, BrickDepth };
        GameObject::CreateInfo gameObjectCreateInfo { };
        gameObjectCreateInfo.pBoxCreateInfo = &gameObjectBoxCreateInfo;
        gameObjectCreateInfo.rigidBodyCreateInfo.mass = BrickMass;
        gameObjectCreateInfo.rigidBodyCreateInfo.initialTransform.setOrigin(BrickPositions[i]);
        gameObjectFactory.create_game_object(gvkContext.get<gvk::CommandBuffers>()[0], gameObjectCreateInfo, &bricks[i]);
        bricks[i].color = BrickRowColors[i / BrickColumCount];
        physicsWorld.make_static(bricks[i].rigidBody);
        liveBricks.insert(&bricks[i]);
    }

    // Create the balls.
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
        gameObjectFactory.create_game_object(gvkContext.get<gvk::CommandBuffers>()[0], gameObjectCreateInfo, &balls[i]);
        balls[i].color = gvk::math::Color::SlateGray;
    }

    // Create the paddle.
    constexpr btScalar PaddleWidth           = 6;
    constexpr btScalar PaddleHeight          = 1;
    constexpr btScalar PaddleDepth           = 0.1f;
    constexpr btScalar PaddleMass            = 1;
    constexpr btScalar PaddleForceStrength   = 48;
    constexpr btScalar PaddleImpulseStrength = 64;
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
        gameObjectFactory.create_game_object(gvkContext.get<gvk::CommandBuffers>()[0], gameObjectCreateInfo, &paddle);
        paddle.color = gvk::math::Color::Brown;
        physicsWorld.make_dynamic(paddle.rigidBody);
    }

    // Create a Clock, some timers that will be used when GameState changes and set
    //  the GameState to GameState::Playing.
    gvk::system::Clock clock;
    float celebrationTimer = 0;
    float resetTimer = 0;
    GameState state = GameState::Playing;

    // Loop until the user presses [Esc] or closes the app window.
    while (
        !(gvkSystemSurface.get<gvk::system::Input>().keyboard.down(gvk::system::Key::Escape)) &&
        !(gvkSystemSurface.get<gvk::system::Surface::StatusFlags>() & gvk::system::Surface::CloseRequested)) {

        // Update the Clock and get the time (in seconds, represented as a float)
        //  elapsed since the last call to clock.update().
        clock.update();
        auto deltaTime = clock.elapsed<gvk::system::Seconds<float>>();

        // Call the static function gvk::system::Surface::update() to cause all
        //  gvk::system::Surface objects to process window/input events.  Get a
        //  reference to the Surface's Input object.
        gvk::system::Surface::update();
        const auto& input = gvkSystemSurface.get<gvk::system::Input>();

        // If the user pressed [~], toggle polygon/wireframe rendering
        if (input.keyboard.pressed(gvk::system::Key::OEM_Tilde)) {
            if (pipeline == polygonPipeline) {
                pipeline = wireframePipeline;
            } else {
                pipeline = polygonPipeline;
            }
        }

        // Update the Camera based on user input.
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

        // Apply a force on the paddle when the user is pressing [<] or [>].
        if (input.keyboard.down(gvk::system::Key::LeftArrow)) {
            paddle.rigidBody.apply_force({ PaddleForceStrength, 0, 0 });
        }
        if (input.keyboard.down(gvk::system::Key::RightArrow)) {
            paddle.rigidBody.apply_force({ -PaddleForceStrength, 0, 0 });
        }

        // Switch on GameState.
        switch (state) {
        case GameState::Playing: {
            // If the user presses [Space] and there's at least one live ball left, fire a
            //  ball.
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
            }
            // If a live brick has been hit by anything (a ball or another brick), make it
            //  dynamic and remove it from the liveBricks collection.
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
            // Check for live balls and ball/paddle collisions.  If there was a collision
            //  between a ball and the top of the paddle, apply an impulse to the ball.
            //  The direction of the impulse is the vector from the center of the paddle to
            //  the center of the ball.
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
                            impulse *= PaddleImpulseStrength;
                            impulse.setY(PaddleImpulseStrength);
                            ball.rigidBody.apply_impulse(impulse);
                        }
                    }
                }
            }
            // If liveBricks is empty, then all bricks have been broken out and GameState
            //  transitions to Celebration.  If there are still live bricks left but no
            //  live balls, then transition GameState to GameOver.
            if (liveBricks.empty()) {
                celebrationTimer = 0;
                state = GameState::Celebration;
            } else if (!liveBallCount) {
                state = GameState::GameOver;
            }
        } break;
        case GameState::Celebration: {
            // Cycle through flashing colors on the play field barriers for some time.
            //  When celebrationTimer reaches CelebrationDuration, reset the play field
            //  barrier colors to Color::White and transition GameState to GameOver.
            constexpr float CelebrationDuration      = 4.5f;
            constexpr float CelebrationColorDuration = 0.01f;
            static const std::vector<glm::vec4> CelebrationColors {
                gvk::math::Color::Red,
                gvk::math::Color::White,
                gvk::math::Color::Blue,
                gvk::math::Color::Yellow
            };
            celebrationTimer += deltaTime;
            if (celebrationTimer < CelebrationDuration) {
                float t = celebrationTimer / CelebrationColorDuration;
                auto colorIndex = (size_t)std::round(t) % CelebrationColors.size();
                for (auto& wall : playFieldBarriers) {
                    wall.color = CelebrationColors[colorIndex];
                }
            } else {
                for (auto& wall : playFieldBarriers) {
                    wall.color = gvk::math::Color::White;
                }
                state = GameState::GameOver;
            }
        } break;
        case GameState::GameOver: {
            // halt() and disable() the bricks.  Record brick positions and rotations for
            //  use in the reset animation.
            for (uint32_t i = 0; i < BrickCount; ++i) {
                auto& brick = bricks[i];
                brick.rigidBody.halt();
                physicsWorld.disable(brick.rigidBody);
                const auto& transform = brick.rigidBody.get_transform();
                std::get<btVector3>(deadBricks[i]) = transform.getOrigin();
                std::get<btQuaternion>(deadBricks[i]) = transform.getRotation();
            }
            // halt() and disable() the balls.
            for (auto& ball : balls) {
                ball.rigidBody.halt();
                physicsWorld.disable(ball.rigidBody);
            }
            // Transition GameState to Resetting
            resetTimer = 0;
            state = GameState::Resetting;
        } break;
        case GameState::Resetting: {
            constexpr float ResetDuration = 2.5f;
            resetTimer += deltaTime;
            // If resetTimer is less than ResetDuration, run the reset logic, otherwise set
            //  all of the bricks and balls to their initial positions (bricks lerp from
            //  their GameOver positions back to their initial positions, but may not land
            //  at exactly their starting positions...the balls should always get back to
            //  their starting positions via the reset logic unless something causes a very
            //  long delta time in which case a ball's reset may be skipped) and transition
            //  GameState to Playing.
            if (resetTimer < ResetDuration) {
                float t = resetTimer / ResetDuration;
                for (uint32_t i = 0; i < BrickCount; ++i) {
                    auto transform = bricks[i].rigidBody.get_transform();
                    transform.setOrigin(std::get<btVector3>(deadBricks[i]).lerp(BrickPositions[i], t));
                    transform.setRotation(std::get<btQuaternion>(deadBricks[i]).slerp(btQuaternion::getIdentity(), t));
                    bricks[i].rigidBody.set_transform(transform);
                }
                // When resetting the balls, a counter is used that starts at BallCount and
                //  counts down.  As the counter decrements, the counter value (minus 1) is
                //  subracted from BallCount to get the index of the ball that should be reset
                //  to its initial position.
                auto ballResetCounter = BallCount - (size_t)(t * (BallCount + 1));
                if (ballResetCounter < BallCount) {
                    auto ballIndex = BallCount - ballResetCounter - 1;
                    auto transform = balls[ballIndex].rigidBody.get_transform();
                    transform.setOrigin(BallPositions[ballIndex]);
                    balls[ballIndex].rigidBody.set_transform(transform);
                }
            } else {
                for (uint32_t i = 0; i < BrickCount; ++i) {
                    auto transform = bricks[i].rigidBody.get_transform();
                    transform.setOrigin(BrickPositions[i]);
                    transform.setRotation(btQuaternion::getIdentity());
                    bricks[i].rigidBody.set_transform(transform);
                    physicsWorld.make_static(bricks[i].rigidBody);
                    liveBricks.insert(&bricks[i]);
                }
                for (uint32_t i = 0; i < BallCount; ++i) {
                    auto transform = balls[i].rigidBody.get_transform();
                    transform.setOrigin(BallPositions[i]);
                    transform.setRotation(btQuaternion::getIdentity());
                    balls[i].rigidBody.set_transform(transform);
                }
                state = GameState::Playing;
            }
        } break;
        }

        // Clamp the paddle's position within the play field boundaries.  This prevents
        //  the paddle from tunneling through the walls.  Without this, the right
        //  combination of forces from the player, the balls, and bricks can cause the
        //  paddle to escape the play field.
        auto paddleTransform = paddle.rigidBody.get_transform();
        auto paddleX = paddleTransform.getOrigin().x();
        auto xMax = PlayFieldWidth * 0.5f - PaddleWidth * 0.5f;
        auto xMin = -xMax;
        paddleTransform.getOrigin().setX(glm::clamp(paddleX, xMin, xMax));
        paddle.rigidBody.set_transform(paddleTransform);

        // Update the dst::physics::World
        physicsWorld.update(deltaTime);

        // Update GameObject uniform buffers
        paddle.update_uniform_buffer(gvkContext.get<gvk::Devices>()[0]);
        for (const auto& wall : playFieldBarriers) {
            wall.update_uniform_buffer(gvkContext.get<gvk::Devices>()[0]);
        }
        for (const auto& brick : bricks) {
            brick.update_uniform_buffer(gvkContext.get<gvk::Devices>()[0]);
        }
        for (const auto& ball : balls) {
            ball.update_uniform_buffer(gvkContext.get<gvk::Devices>()[0]);
        }

        // If wireframe (debug) mode is enabled, update the container uniform buffers.
        if (pipeline == wireframePipeline) {
            for (const auto& containerBarrier : containerBarriers) {
                containerBarrier.update_uniform_buffer(gvkContext.get<gvk::Devices>()[0]);
            }
        }

#if 0
        // Call wsiManager.update().  This will cause WsiManager to respond to system
        //  updates for the SurfaceKHR it's managing.  This call may cause resources to
        //  be created/destroyed.  If there's a valid SwapchainKHR, render and present.
        wsiManager.update();
        auto swapchain = wsiManager.get_swapchain();
        if (swapchain) {
            // Update the Camera's aspect ratio to match the SwapchainKHR's then update the
            //  Camera's uniform buffer
            auto extent = wsiManager.get_swapchain().get<VkSwapchainCreateInfoKHR>().imageExtent;
            camera.set_aspect_ratio(extent.width, extent.height);
#else
        // TODO : Documentation
        gvk::wsi::AcquiredImageInfo acquiredImageInfo{ };
        gvk::RenderTarget acquiredImageRenderTarget{ };
        auto wsiStatus = wsiContext.acquire_next_image(UINT64_MAX, VK_NULL_HANDLE, &acquiredImageInfo, &acquiredImageRenderTarget);
        if (wsiStatus == VK_SUCCESS || wsiStatus == VK_SUBOPTIMAL_KHR) {
            auto extent = wsiContext.get<gvk::SwapchainKHR>().get<VkSwapchainCreateInfoKHR>().imageExtent;
            camera.set_aspect_ratio(extent.width, extent.height);
#endif

            CameraUniforms cameraUbo { };
            cameraUbo.view = camera.view();
            cameraUbo.projection = camera.projection();
            VmaAllocationInfo allocationInfo { };
            vmaGetAllocationInfo(gvkContext.get<gvk::Devices>()[0].get<VmaAllocator>(), cameraUniformBuffer.get<VmaAllocation>(), &allocationInfo);
            memcpy(allocationInfo.pMappedData, &cameraUbo, sizeof(CameraUniforms));

#if 0
            // Acquire the next Image to render to.  The index will be used to access the
            //  acquired Image as well as the CommandBuffer and Fence associated with that
            //  Image.  Note that this method may return VK_SUBOPTIMAL_KHR when the window
            //  is resized/minimized/maximized/etc...gvk::WsiManager will update resources
            //  when this occurs so there's no need to bail.
            uint32_t imageIndex = 0;
            auto vkResult = wsiManager.acquire_next_image(UINT64_MAX, VK_NULL_HANDLE, &imageIndex);
            assert(vkResult == VK_SUCCESS || vkResult == VK_SUBOPTIMAL_KHR);

            // Using the acquired Image index, wait on the associated Fence.  This ensures
            //  that the Image isn't currently in use via vkQueueSubmit().  After waiting
            //  on the Fence, immediately reset it so that it's ready to be used in the
            //  next call to vkQueueSubmit().
            const auto& vkFences = wsiManager.get_vk_fences();
            dst_vk_result(vkWaitForFences(gvkDevice, 1, &vkFences[imageIndex], VK_TRUE, UINT64_MAX));
            dst_vk_result(vkResetFences(gvkDevice, 1, &vkFences[imageIndex]));
#endif

            // Begin CommandBuffer recording and begin a RenderPass.
            dst_vk_result(vkBeginCommandBuffer(acquiredImageInfo.commandBuffer, &gvk::get_default<VkCommandBufferBeginInfo>()));
            const auto& renderPassBeginInfo = acquiredImageRenderTarget.get<VkRenderPassBeginInfo>();
            vkCmdBeginRenderPass(acquiredImageInfo.commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

            // Set the scissor and viewport to match the renderPassBeginInfo.renderArea
            VkRect2D scissor{ { }, renderPassBeginInfo.renderArea.extent };
            vkCmdSetScissor(acquiredImageInfo.commandBuffer, 0, 1, &scissor);
            VkViewport viewport{ 0, 0, (float)scissor.extent.width, (float)scissor.extent.height, 0, 1 };
            vkCmdSetViewport(acquiredImageInfo.commandBuffer, 0, 1, &viewport);

            // Bind the current Pipeline
            vkCmdBindPipeline(acquiredImageInfo.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

            // Bind the Camera's DescriptorSet.  Since all objects are being drawn with the
            //  same Camera this binding will be used for all subsequent draw calls in this
            //  RenderPass.
            const auto& pipelineLayout = pipeline.get<gvk::PipelineLayout>();
            vkCmdBindDescriptorSets(acquiredImageInfo.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &cameraDescriptorSet.get<VkDescriptorSet>(), 0, nullptr);

            // Record draw calls for all of the GameObjects.
            paddle.record_draw_cmds(acquiredImageInfo.commandBuffer, pipelineLayout);
            for (const auto& playFieldBarrier : playFieldBarriers) {
                playFieldBarrier.record_draw_cmds(acquiredImageInfo.commandBuffer, pipelineLayout);
            }
            for (const auto& brick : bricks) {
                brick.record_draw_cmds(acquiredImageInfo.commandBuffer, pipelineLayout);
            }
            for (const auto& ball : balls) {
                ball.record_draw_cmds(acquiredImageInfo.commandBuffer, pipelineLayout);
            }

            // If wireframe (debug) mode is enabled, draw the container.
            if (pipeline == wireframePipeline) {
                for (const auto& containerBarrier : containerBarriers) {
                    containerBarrier.record_draw_cmds(acquiredImageInfo.commandBuffer, pipelineLayout);
                }
            }

            // End the RenderPass and CommandBuffer.
            vkCmdEndRenderPass(acquiredImageInfo.commandBuffer);
            dst_vk_result(vkEndCommandBuffer(acquiredImageInfo.commandBuffer));

            // Submit the CommandBuffer for execution on the GPU.
            const auto& queue = gvk::get_queue_family(gvkContext.get<gvk::Devices>()[0], 0).queues[0];
            dst_vk_result(vkQueueSubmit(queue, 1, &wsiContext.get<VkSubmitInfo>(acquiredImageInfo), acquiredImageInfo.fence));

            // Present the SwapchainKHR Image that was drawn into.
            wsiStatus = wsiContext.queue_present(queue, &acquiredImageInfo);
            dst_vk_result((wsiStatus == VK_SUBOPTIMAL_KHR || wsiStatus == VK_ERROR_OUT_OF_DATE_KHR) ? VK_SUCCESS : wsiStatus);
        }
    }

    // Wait for the GPU to be idle before allowing graphics destructors to run.
    dst_vk_result(vkDeviceWaitIdle(gvkContext.get<gvk::Devices>()[0]));

    // Reset the dst::physics::World before allowing physics destructors to run.
    physicsWorld.reset();

    return 0;
}
