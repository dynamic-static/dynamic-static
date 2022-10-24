
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

#include <iostream>
#include <map>
#include <memory>
#include <vector>

void create_icosphere_mesh_data(float radius, uint32_t subdivisions, std::vector<glm::vec3>* pVertices, std::vector<glm::uvec3>* pFaces)
{
    (void)radius;
    assert(pVertices);
    assert(pFaces);
    // FROM : https://gitlab.com/libeigen/eigen/-/blob/master/demos/opengl/icosphere.cpp
    static const float X = 0.525731112119133606f;
    static const float Z = 0.850650808352039932f;
    static const std::array<glm::vec3, 12> Vertices {
        glm::vec3 { -X, 0, Z }, glm::vec3 {  X, 0,  Z }, glm::vec3 { -X,  0, -Z }, glm::vec3 {  X,  0, -Z },
        glm::vec3 {  0, Z, X }, glm::vec3 {  0, Z, -X }, glm::vec3 {  0, -Z,  X }, glm::vec3 {  0, -Z, -X },
        glm::vec3 {  Z, X, 0 }, glm::vec3 { -Z, X,  0 }, glm::vec3 {  Z, -X,  0 }, glm::vec3 { -Z, -X,  0 }
    };
    static const std::array<glm::u32vec3, 20> Faces {
        glm::u32vec3 { 0,  4,  1 }, glm::u32vec3 { 0, 9,  4 }, glm::u32vec3 { 9,  5, 4 }, glm::u32vec3 {  4, 5, 8 }, glm::u32vec3 { 4, 8,  1 },
        glm::u32vec3 { 8, 10,  1 }, glm::u32vec3 { 8, 3, 10 }, glm::u32vec3 { 5,  3, 8 }, glm::u32vec3 {  5, 2, 3 }, glm::u32vec3 { 2, 7,  3 },
        glm::u32vec3 { 7, 10,  3 }, glm::u32vec3 { 7, 6, 10 }, glm::u32vec3 { 7, 11, 6 }, glm::u32vec3 { 11, 0, 6 }, glm::u32vec3 { 0, 1,  6 },
        glm::u32vec3 { 6,  1, 10 }, glm::u32vec3 { 9, 0, 11 }, glm::u32vec3 { 9, 11, 2 }, glm::u32vec3 {  9, 2, 5 }, glm::u32vec3 { 7, 2, 11 },
    };
    pVertices->clear();
    pVertices->reserve(Vertices.size());
    pVertices->insert(pVertices->end(), Vertices.begin(), Vertices.end());
    pFaces->clear();
    pFaces->reserve(Faces.size());
    pFaces->insert(pFaces->end(), Faces.begin(), Faces.end());
    for (; 0 < subdivisions; --subdivisions) {

    }
}

VkResult create_icosphere_mesh(const gvk::Context& context, float radius, uint32_t subdivisions, const glm::vec4& color, gvk::Mesh* pMesh)
{
    std::vector<glm::vec3> positions;
    std::vector<glm::u32vec3> faces;
    create_icosphere_mesh_data(radius, subdivisions, &positions, &faces);
    std::vector<dst::gfx::VertexPositionNormalColor> vertices;
    vertices.reserve(positions.size());
    for (const auto& position : positions) {
        vertices.emplace_back();
        vertices.back().position = position;
        vertices.back().normal = glm::normalize(position);
        vertices.back().color = color;
    }
    std::vector<uint32_t> indices;
    indices.reserve(faces.size() * 3);
    for (const auto& face : faces) {
        indices.push_back(face[0]);
        indices.push_back(face[1]);
        indices.push_back(face[2]);
    }
    return pMesh->write(
        context.get_devices()[0],
        gvk::get_queue_family(context.get_devices()[0], 0).queues[0],
        context.get_command_buffers()[0],
        VK_NULL_HANDLE,
        (uint32_t)vertices.size(),
        vertices.data(),
        (uint32_t)indices.size(),
        indices.data()
    );
}

VkResult create_box_mesh(const gvk::Context& context, const glm::vec3& dimensions, const glm::vec4& color, gvk::Mesh* pMesh)
{
    float w = dimensions[0] * 0.5f;
    float h = dimensions[1] * 0.5f;
    float d = dimensions[2] * 0.5f;
    static const glm::vec3 UnitX { 1, 0, 0 };
    static const glm::vec3 UnitY { 0, 1, 0 };
    static const glm::vec3 UnitZ { 0, 0, 1 };
    std::array<dst::gfx::VertexPositionNormalColor, 24> vertices {
        // Top
        dst::gfx::VertexPositionNormalColor { { -w,  h, -d }, {  UnitY }, { color } },
        dst::gfx::VertexPositionNormalColor { {  w,  h, -d }, {  UnitY }, { color } },
        dst::gfx::VertexPositionNormalColor { {  w,  h,  d }, {  UnitY }, { color } },
        dst::gfx::VertexPositionNormalColor { { -w,  h,  d }, {  UnitY }, { color } },
        // Left
        dst::gfx::VertexPositionNormalColor { { -w,  h, -d }, { -UnitX }, { color } },
        dst::gfx::VertexPositionNormalColor { { -w,  h,  d }, { -UnitX }, { color } },
        dst::gfx::VertexPositionNormalColor { { -w, -h,  d }, { -UnitX }, { color } },
        dst::gfx::VertexPositionNormalColor { { -w, -h, -d }, { -UnitX }, { color } },
        // Front
        dst::gfx::VertexPositionNormalColor { { -w,  h,  d }, {  UnitZ }, { color } },
        dst::gfx::VertexPositionNormalColor { {  w,  h,  d }, {  UnitZ }, { color } },
        dst::gfx::VertexPositionNormalColor { {  w, -h,  d }, {  UnitZ }, { color } },
        dst::gfx::VertexPositionNormalColor { { -w, -h,  d }, {  UnitZ }, { color } },
        // Right
        dst::gfx::VertexPositionNormalColor { {  w,  h,  d }, {  UnitX }, { color } },
        dst::gfx::VertexPositionNormalColor { {  w,  h, -d }, {  UnitX }, { color } },
        dst::gfx::VertexPositionNormalColor { {  w, -h, -d }, {  UnitX }, { color } },
        dst::gfx::VertexPositionNormalColor { {  w, -h,  d }, {  UnitX }, { color } },
        // Back
        dst::gfx::VertexPositionNormalColor { {  w,  h, -d }, { -UnitZ }, { color } },
        dst::gfx::VertexPositionNormalColor { { -w,  h, -d }, { -UnitZ }, { color } },
        dst::gfx::VertexPositionNormalColor { { -w, -h, -d }, { -UnitZ }, { color } },
        dst::gfx::VertexPositionNormalColor { {  w, -h, -d }, { -UnitZ }, { color } },
        // Bottom
        dst::gfx::VertexPositionNormalColor { { -w, -h,  d }, { -UnitY }, { color } },
        dst::gfx::VertexPositionNormalColor { {  w, -h,  d }, { -UnitY }, { color } },
        dst::gfx::VertexPositionNormalColor { {  w, -h, -d }, { -UnitY }, { color } },
        dst::gfx::VertexPositionNormalColor { { -w, -h, -d }, { -UnitY }, { color } },
    };
    size_t index_i = 0;
    size_t vertex_i = 0;
    constexpr size_t FaceCount = 6;
    constexpr size_t IndicesPerFace = 6;
    std::array<uint16_t, IndicesPerFace* FaceCount> indices;
    for (size_t face_i = 0; face_i < FaceCount; ++face_i) {
        indices[index_i++] = (uint16_t)(vertex_i + 0);
        indices[index_i++] = (uint16_t)(vertex_i + 1);
        indices[index_i++] = (uint16_t)(vertex_i + 2);
        indices[index_i++] = (uint16_t)(vertex_i + 2);
        indices[index_i++] = (uint16_t)(vertex_i + 3);
        indices[index_i++] = (uint16_t)(vertex_i + 0);
        vertex_i += 4;
    }
    return pMesh->write(
        context.get_devices()[0],
        gvk::get_queue_family(context.get_devices()[0], 0).queues[0],
        context.get_command_buffers()[0],
        VK_NULL_HANDLE,
        (uint32_t)vertices.size(),
        vertices.data(),
        (uint32_t)indices.size(),
        indices.data()
    );
}

struct ObjectUniforms
{
    glm::mat4 world { };
};

struct CameraUniforms
{
    glm::mat4 view { };
    glm::mat4 projection { };
    glm::vec3 position { };
};

int main(int, const char* [])
{
    ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

    dynamicsWorld->setGravity(btVector3(0, -9.80665f, 0));

    ///-----initialization_end-----

    //keep track of the shapes, we release memory at exit.
    //make sure to re-use collision shapes among rigid bodies whenever possible!
    btAlignedObjectArray<btCollisionShape*> collisionShapes;

    ///create a few basic rigid bodies

    btRigidBody* pBoxRigidBody = nullptr;
    btRigidBody* pGroundRigidBody = nullptr;

    //the ground is a cube of side 100 at position y = -56.
    //the sphere will hit it at y = -6, with center at -5
    {
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

        collisionShapes.push_back(groundShape);

        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -56, 0));

        btScalar mass(0.);

        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            groundShape->calculateLocalInertia(mass, localInertia);

        //using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        //add the body to the dynamics world
        dynamicsWorld->addRigidBody(body);
        pGroundRigidBody = body;
    }

    {
        //create a dynamic rigidbody

        btCollisionShape* colShape = new btBoxShape(btVector3(1, 1, 1));
        // btCollisionShape* colShape = new btSphereShape(btScalar(1.));
        collisionShapes.push_back(colShape);

        /// Create Dynamic Objects
        btTransform startTransform;
        startTransform.setIdentity();

        btScalar mass(1.f);

        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            colShape->calculateLocalInertia(mass, localInertia);

        startTransform.setOrigin(btVector3(2, 10, 0));

        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        dynamicsWorld->addRigidBody(body);
        pBoxRigidBody = body;
    }

    GfxContext gfxContext;
    auto vkResult = GfxContext::create("dynamic-static - block-blaster", &gfxContext);
    assert(vkResult == VK_SUCCESS);

    gvk::Pipeline pipeline;

    gvk::Mesh boxMesh;
    gvk::Buffer boxUniformBuffer;
    gvk::DescriptorSet boxDescriptorSet;

    gvk::Mesh groundMesh;
    gvk::Buffer groundUniformBuffer;
    gvk::DescriptorSet groundDescriptorSet;

    gvk::math::Camera camera;
    gvk::Buffer cameraUniformBuffer;
    gvk::DescriptorSet cameraDescriptorSet;
    gvk::math::FreeCameraController cameraController;
    cameraController.set_camera(&camera);
    camera.transform.translation.z = -32;

    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED)
    {
        gvk_result(create_icosphere_mesh(gfxContext, 1, 1, gvk::math::Color::Blue, &boxMesh));
        // gvk_result(create_box_mesh(gfxContext, { 2, 2, 2 }, gvk::math::Color::OrangeRed, &boxMesh));
        gvk_result(create_box_mesh(gfxContext, { 100, 100, 100 }, gvk::math::Color::AntiqueWhite, &groundMesh));
        gvk_result(dst_sample_create_uniform_buffer<ObjectUniforms>(gfxContext, &boxUniformBuffer));
        gvk_result(dst_sample_create_uniform_buffer<ObjectUniforms>(gfxContext, &groundUniformBuffer));
        gvk_result(dst_sample_create_uniform_buffer<CameraUniforms>(gfxContext, &cameraUniformBuffer));
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
                    vec3 position;
                } camera;

                layout(set = 1, binding = 0)
                uniform ObjectUniformBuffer
                {
                    mat4 world;
                } object;

                layout(location = 0) in vec3 vsPosition;
                layout(location = 1) in vec3 vsNormal;
                layout(location = 2) in vec4 vsColor;

                layout(location = 0) out vec3 fsPosition;
                layout(location = 1) out vec3 fsNormal;
                layout(location = 2) out vec4 fsColor;
                layout(location = 3) out vec3 fsCameraPosition;

                out gl_PerVertex
                {
                    vec4 gl_Position;
                };

                void main()
                {
                    fsPosition = vsPosition;
                    fsNormal = vsNormal;
                    fsColor = vsColor;
                    fsCameraPosition = camera.position;
                    gl_Position = camera.projection * camera.view * object.world * vec4(vsPosition, 1);
                }
            )"
        };
        gvk::spirv::ShaderInfo fragmentShaderInfo {
            .language = gvk::spirv::ShadingLanguage::Glsl,
            .stage = VK_SHADER_STAGE_FRAGMENT_BIT,
            .lineOffset = __LINE__,
            .source = R"(
                #version 450

                layout(location = 0) in vec3 fsPosition;
                layout(location = 1) in vec3 fsNormal;
                layout(location = 2) in vec4 fsColor;
                layout(location = 3) in vec3 fsCameraPosition;

                layout(location = 0) out vec4 fragColor;

                void main()
                {
                    vec3 lightPosition = vec3(0, 100, 0);
                    vec3 lightDirection = normalize(lightPosition - fsPosition);
                    vec3 normal = normalize(fsNormal);
                    vec3 ambient = fsColor.rgb * 0.25;
                    vec3 diffuse = vec3(dot(lightDirection, normal));
                    vec3 viewDirection = normalize(fsCameraPosition - fsPosition);
                    vec3 reflectionDirection = reflect(-lightDirection, normal);
                    vec3 specular = vec3(0.3) * pow(max(0, dot(viewDirection, reflectionDirection)), 8);
                    fragColor = vec4(ambient + diffuse + specular, 1);
                }
            )"
        };

        gvk_result(dst_sample_create_pipeline<dst::gfx::VertexPositionNormalColor>(
            gfxContext.get_wsi_manager().get_render_pass(),
            VK_CULL_MODE_NONE,
            vertexShaderInfo,
            fragmentShaderInfo,
            &pipeline
        ));

        std::vector<gvk::DescriptorSet> descriptorSets;
        gvk_result(dst_sample_allocate_descriptor_sets(pipeline, &descriptorSets));
        assert(descriptorSets.size() == 2);
        cameraDescriptorSet = descriptorSets[0];
        boxDescriptorSet = descriptorSets[1];
        gvk_result(dst_sample_allocate_descriptor_sets(pipeline, &descriptorSets));
        assert(descriptorSets.size() == 2);
        groundDescriptorSet = descriptorSets[1];

        auto boxUniformBufferDescriptorInfo = gvk::get_default<VkDescriptorBufferInfo>();
        boxUniformBufferDescriptorInfo.buffer = boxUniformBuffer;
        auto groundUniformBufferDescriptorInfo = gvk::get_default<VkDescriptorBufferInfo>();
        groundUniformBufferDescriptorInfo.buffer = groundUniformBuffer;
        auto cameraUniformBufferDescriptorInfo = gvk::get_default<VkDescriptorBufferInfo>();
        cameraUniformBufferDescriptorInfo.buffer = cameraUniformBuffer;

        // Write the descriptors...
        std::array<VkWriteDescriptorSet, 3> writeDescriptorSets {
            VkWriteDescriptorSet {
                .sType = gvk::get_stype<VkWriteDescriptorSet>(),
                .dstSet = boxDescriptorSet,
                .dstBinding = 0,
                .descriptorCount = 1,
                .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                .pBufferInfo = &boxUniformBufferDescriptorInfo,
            },
            VkWriteDescriptorSet {
                .sType = gvk::get_stype<VkWriteDescriptorSet>(),
                .dstSet = groundDescriptorSet,
                .dstBinding = 0,
                .descriptorCount = 1,
                .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                .pBufferInfo = &groundUniformBufferDescriptorInfo,
            },
            VkWriteDescriptorSet {
                .sType = gvk::get_stype<VkWriteDescriptorSet>(),
                .dstSet = cameraDescriptorSet,
                .dstBinding = 0,
                .descriptorCount = 1,
                .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                .pBufferInfo = &cameraUniformBufferDescriptorInfo,
            },
        };
        vkUpdateDescriptorSets(gfxContext.get_devices()[0], (uint32_t)writeDescriptorSets.size(), writeDescriptorSets.data(), 0, nullptr);
    } gvk_result_scope_end;

    /// Do some simulation

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

        CameraUniforms cameraUbo { };
        cameraUbo.view = camera.view();
        cameraUbo.projection = camera.projection();
        cameraUbo.position = camera.transform.translation;
        VmaAllocationInfo allocationInfo { };
        vmaGetAllocationInfo(gfxContext.get_devices()[0].get<VmaAllocator>(), cameraUniformBuffer.get<VmaAllocation>(), &allocationInfo);
        assert(allocationInfo.pMappedData);
        memcpy(allocationInfo.pMappedData, &cameraUbo, sizeof(CameraUniforms));

        dynamicsWorld->stepSimulation(deltaTime, 10);

        btTransform btTransform;
        pBoxRigidBody->getMotionState()->getWorldTransform(btTransform);
        ObjectUniforms boxUbo { };
        btTransform.getOpenGLMatrix(&boxUbo.world[0][0]);
        vmaGetAllocationInfo(gfxContext.get_devices()[0].get<VmaAllocator>(), boxUniformBuffer.get<VmaAllocation>(), &allocationInfo);
        assert(allocationInfo.pMappedData);
        memcpy(allocationInfo.pMappedData, &boxUbo, sizeof(ObjectUniforms));

        pGroundRigidBody->getMotionState()->getWorldTransform(btTransform);
        ObjectUniforms groundUbo { };
        btTransform.getOpenGLMatrix(&groundUbo.world[0][0]);
        vmaGetAllocationInfo(gfxContext.get_devices()[0].get<VmaAllocator>(), groundUniformBuffer.get<VmaAllocation>(), &allocationInfo);
        assert(allocationInfo.pMappedData);
        memcpy(allocationInfo.pMappedData, &groundUbo, sizeof(ObjectUniforms));

        float x = camera.transform.translation.x;
        float y = camera.transform.translation.y;
        float z = camera.transform.translation.z;
        std::cout << x << ", " << y << ", " << z << std::endl;

        auto& wsiManager = gfxContext.get_wsi_manager();
        if (wsiManager.update()) {
            auto extent = wsiManager.get_swapchain().get<VkSwapchainCreateInfoKHR>().imageExtent;
            camera.set_aspect_ratio(extent.width, extent.height);
            for (size_t i = 0; i < wsiManager.get_command_buffers().size(); ++i) {
                const auto& commandBuffer = wsiManager.get_command_buffers()[i];
                vkResult = vkBeginCommandBuffer(commandBuffer, &gvk::get_default<VkCommandBufferBeginInfo>());
                assert(vkResult == VK_SUCCESS);

                // Begin the gvk::RenderPass that renders into the gvk::WsiManager...
                auto renderPassBeginInfo = wsiManager.get_render_targets()[i].get_render_pass_begin_info();
                vkCmdBeginRenderPass(commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
                {
                    VkRect2D scissor { .extent = renderPassBeginInfo.renderArea.extent };
                    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
                    VkViewport viewport { .width = (float)scissor.extent.width, .height = (float)scissor.extent.height, .minDepth = 0, .maxDepth = 1 };
                    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

                    auto pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
                    vkCmdBindPipeline(commandBuffer, pipelineBindPoint, pipeline);
                    vkCmdBindDescriptorSets(commandBuffer, pipelineBindPoint, pipeline.get<gvk::PipelineLayout>(), 0, 1, &(const VkDescriptorSet&)cameraDescriptorSet, 0, nullptr);
                    vkCmdBindDescriptorSets(commandBuffer, pipelineBindPoint, pipeline.get<gvk::PipelineLayout>(), 1, 1, &(const VkDescriptorSet&)groundDescriptorSet, 0, nullptr);
                    groundMesh.record_cmds(commandBuffer);
                    vkCmdBindDescriptorSets(commandBuffer, pipelineBindPoint, pipeline.get<gvk::PipelineLayout>(), 1, 1, &(const VkDescriptorSet&)boxDescriptorSet, 0, nullptr);
                    boxMesh.record_cmds(commandBuffer);
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
    for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState()) {
            delete body->getMotionState();
        }
        dynamicsWorld->removeCollisionObject(obj);
        delete obj;
    }

    //delete collision shapes
    for (int j = 0; j < collisionShapes.size(); j++) {
        btCollisionShape* shape = collisionShapes[j];
        collisionShapes[j] = 0;
        delete shape;
    }

    //delete dynamics world
    delete dynamicsWorld;

    //delete solver
    delete solver;

    //delete broadphase
    delete overlappingPairCache;

    //delete dispatcher
    delete dispatcher;

    delete collisionConfiguration;

    //next line is optional: it will be cleared by the destructor when the array goes out of scope
    collisionShapes.clear();

    return 0;
}
