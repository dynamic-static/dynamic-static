
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

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

VkResult create_icosphere_mesh(const gvk::Context& context, float radius, uint32_t subdivisions, const glm::vec4& color, gvk::Mesh* pMesh)
{
    // TODO : Documentation
    std::vector<dst::gfx::VertexPositionNormalColor> vertices;
    auto createVertex = [&](const glm::vec3& vertex)
    {
        vertices.push_back({ });
        vertices.back().position = vertex * radius;
        vertices.back().normal = vertex;
        vertices.back().color = color;
        return (uint32_t)vertices.size() - 1;
    };

    // TODO : Documentation
    for (const auto& vertex : dst::gfx::primitive::Icosahedron::Vertices) {
        createVertex(vertex);
    }

    // TODO : Documentation
    const auto& icosahedronTriangles = dst::gfx::primitive::Icosahedron::Triangles;
    std::vector<dst::gfx::primitive::Triangle<uint32_t>> triangles;
    triangles.insert(triangles.end(), icosahedronTriangles.begin(), icosahedronTriangles.end());

    // TODO : Documentation
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
                        auto v0 = vertices[edge[0]].position;
                        auto v1 = vertices[edge[1]].position;
                        auto vertex = glm::normalize((v0 + v1) * 0.5f);
                        auto index = createVertex(vertex);
                        itr = edges.insert(itr, { edge, index });
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

    // TODO : Documentation
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
#endif
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

class Object final
{
public:
    void update_uniform_buffer(const VmaAllocator& vmaAllocator)
    {
        btTransform btTransform { };
        upRigidBody->getMotionState()->getWorldTransform(btTransform);
        ObjectUniforms boxUbo { };
        btTransform.getOpenGLMatrix(&boxUbo.world[0][0]);
        VmaAllocationInfo allocationInfo { };
        vmaGetAllocationInfo(vmaAllocator, uniformBuffer.get<VmaAllocation>(), &allocationInfo);
        assert(allocationInfo.pMappedData);
        memcpy(allocationInfo.pMappedData, &boxUbo, sizeof(ObjectUniforms));
    }

    gvk::Mesh mesh;
    gvk::Buffer uniformBuffer;
    gvk::DescriptorSet descriptorSet;
    std::unique_ptr<btMotionState> upMotionState;
    std::unique_ptr<btRigidBody> upRigidBody;
    std::unique_ptr<btCollisionShape> upCollisionShape;
};

int main(int, const char* [])
{
    dst::physics::Context physicsContext;
    dst::physics::Context::create(&physicsContext);

    GfxContext gfxContext;
    auto vkResult = GfxContext::create("dynamic-static - block-blaster", &gfxContext);
    assert(vkResult == VK_SUCCESS);

    gvk::Pipeline pipeline;

    Object sphere;
    {
        btScalar mass = 1;
        btVector3 inertia(0, 0, 0);
        btTransform transform { };
        transform.setIdentity();
        transform.setOrigin(btVector3(2, 10, 0));

        sphere.upCollisionShape = std::make_unique<btSphereShape>((btScalar)1);
        sphere.upCollisionShape->calculateLocalInertia(mass, inertia);
        sphere.upMotionState = std::make_unique<btDefaultMotionState>(transform);
        sphere.upRigidBody = std::make_unique<btRigidBody>(
            btRigidBody::btRigidBodyConstructionInfo(
                mass, sphere.upMotionState.get(), sphere.upCollisionShape.get(), inertia
            )
        );
        physicsContext.mupWorld->addRigidBody(sphere.upRigidBody.get());
    }

    Object ground;
    {
        btScalar mass = 0;
        btVector3 inertia(0, 0, 0);
        btVector3 origin(0, -56, 0);
        btTransform transform { };
        transform.setIdentity();
        transform.setOrigin(origin);

        ground.upCollisionShape = std::make_unique<btBoxShape>(btVector3((btScalar)50, (btScalar)50, (btScalar)50));
        // Skip calculateLocalInertia() to make this rigid body static
        ground.upMotionState = std::make_unique<btDefaultMotionState>(transform);
        ground.upRigidBody = std::make_unique<btRigidBody>(
            btRigidBody::btRigidBodyConstructionInfo(
                mass, ground.upMotionState.get(), ground.upCollisionShape.get(), inertia
            )
        );
        physicsContext.mupWorld->addRigidBody(ground.upRigidBody.get());
    }

    gvk::math::Camera camera;
    gvk::Buffer cameraUniformBuffer;
    gvk::DescriptorSet cameraDescriptorSet;
    gvk::math::FreeCameraController cameraController;
    cameraController.set_camera(&camera);
    camera.transform.translation.z = -32;

    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED)
    {
        gvk_result(create_icosphere_mesh(gfxContext, 1, 3, gvk::math::Color::DodgerBlue, &sphere.mesh));
        // gvk_result(create_box_mesh(gfxContext, { 2, 2, 2 }, gvk::math::Color::OrangeRed, &boxMesh));
        gvk_result(create_box_mesh(gfxContext, { 100, 100, 100 }, gvk::math::Color::AntiqueWhite, &ground.mesh));
        gvk_result(dst_sample_create_uniform_buffer<ObjectUniforms>(gfxContext, &sphere.uniformBuffer));
        gvk_result(dst_sample_create_uniform_buffer<ObjectUniforms>(gfxContext, &ground.uniformBuffer));
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
fragColor = fsColor;
                }
            )"
        };

        gvk_result(dst_sample_create_pipeline<dst::gfx::VertexPositionNormalColor>(
            gfxContext.get_wsi_manager().get_render_pass(),
            VK_CULL_MODE_BACK_BIT,
            VK_POLYGON_MODE_LINE,
            vertexShaderInfo,
            fragmentShaderInfo,
            &pipeline
        ));

        std::vector<gvk::DescriptorSet> descriptorSets;
        gvk_result(dst_sample_allocate_descriptor_sets(pipeline, &descriptorSets));
        assert(descriptorSets.size() == 2);
        cameraDescriptorSet = descriptorSets[0];
        sphere.descriptorSet = descriptorSets[1];
        gvk_result(dst_sample_allocate_descriptor_sets(pipeline, &descriptorSets));
        assert(descriptorSets.size() == 2);
        ground.descriptorSet = descriptorSets[1];

        auto sphereUniformBufferDescriptorInfo = gvk::get_default<VkDescriptorBufferInfo>();
        sphereUniformBufferDescriptorInfo.buffer = sphere.uniformBuffer;
        auto groundUniformBufferDescriptorInfo = gvk::get_default<VkDescriptorBufferInfo>();
        groundUniformBufferDescriptorInfo.buffer = ground.uniformBuffer;
        auto cameraUniformBufferDescriptorInfo = gvk::get_default<VkDescriptorBufferInfo>();
        cameraUniformBufferDescriptorInfo.buffer = cameraUniformBuffer;

        // Write the descriptors...
        std::array<VkWriteDescriptorSet, 3> writeDescriptorSets {
            VkWriteDescriptorSet {
                .sType = gvk::get_stype<VkWriteDescriptorSet>(),
                .dstSet = sphere.descriptorSet,
                .dstBinding = 0,
                .descriptorCount = 1,
                .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                .pBufferInfo = &sphereUniformBufferDescriptorInfo,
            },
            VkWriteDescriptorSet {
                .sType = gvk::get_stype<VkWriteDescriptorSet>(),
                .dstSet = ground.descriptorSet,
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

        physicsContext.mupWorld->stepSimulation(deltaTime, 10);

        auto vmaAllocator = gfxContext.get_devices()[0].get<VmaAllocator>();
        sphere.update_uniform_buffer(vmaAllocator);
        ground.update_uniform_buffer(vmaAllocator);

        CameraUniforms cameraUbo { };
        cameraUbo.view = camera.view();
        cameraUbo.projection = camera.projection();
        cameraUbo.position = camera.transform.translation;
        VmaAllocationInfo allocationInfo { };
        vmaGetAllocationInfo(vmaAllocator, cameraUniformBuffer.get<VmaAllocation>(), &allocationInfo);
        assert(allocationInfo.pMappedData);
        memcpy(allocationInfo.pMappedData, &cameraUbo, sizeof(CameraUniforms));

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
                    vkCmdBindDescriptorSets(commandBuffer, pipelineBindPoint, pipeline.get<gvk::PipelineLayout>(), 1, 1, &(const VkDescriptorSet&)ground.descriptorSet, 0, nullptr);
                    ground.mesh.record_cmds(commandBuffer);
                    vkCmdBindDescriptorSets(commandBuffer, pipelineBindPoint, pipeline.get<gvk::PipelineLayout>(), 1, 1, &(const VkDescriptorSet&)sphere.descriptorSet, 0, nullptr);
                    sphere.mesh.record_cmds(commandBuffer);
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
