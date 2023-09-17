
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

#include "dynamic-static/text.hpp"
#include "dynamic-static.graphics/text.hpp"

#include "stb/stb_image.h"

#include <array>
#include <cassert>
#include <iostream>
#include <vector>

struct ObjectUniforms
{
    glm::mat4 world{ };
};

struct CameraUniforms
{
    glm::mat4 view{ };
    glm::mat4 projection{ };
};

struct Uniforms
{
    ObjectUniforms object{ };
    CameraUniforms camera{ };
};

VkResult create_mesh(
    const gvk::Context& context,
    const glm::vec3& dimensions,
    const glm::vec4& topColor,
    const glm::vec4& bottomColor,
    gvk::Mesh* pMesh
)
{
    float w = dimensions[0] * 0.5f;
    float h = dimensions[1] * 0.5f;
    float d = dimensions[2] * 0.5f;
    std::array<dst::gfx::VertexPositionTexcoordColor, 24> vertices {
        // Top
        dst::gfx::VertexPositionTexcoordColor {{ -w,  h, -d }, { 0, 0 }, { topColor }},
        dst::gfx::VertexPositionTexcoordColor {{  w,  h, -d }, { 1, 0 }, { topColor }},
        dst::gfx::VertexPositionTexcoordColor {{  w,  h,  d }, { 1, 1 }, { topColor }},
        dst::gfx::VertexPositionTexcoordColor {{ -w,  h,  d }, { 0, 1 }, { topColor }},
        // Left
        dst::gfx::VertexPositionTexcoordColor {{ -w,  h, -d }, { 0, 0 }, { topColor }},
        dst::gfx::VertexPositionTexcoordColor {{ -w,  h,  d }, { 0, 0 }, { topColor }},
        dst::gfx::VertexPositionTexcoordColor {{ -w, -h,  d }, { 0, 0 }, { bottomColor }},
        dst::gfx::VertexPositionTexcoordColor {{ -w, -h, -d }, { 0, 0 }, { bottomColor }},
        // Front
        dst::gfx::VertexPositionTexcoordColor {{ -w,  h,  w }, { 0, 0 }, { topColor }},
        dst::gfx::VertexPositionTexcoordColor {{  w,  h,  w }, { 0, 0 }, { topColor }},
        dst::gfx::VertexPositionTexcoordColor {{  w, -h,  w }, { 0, 0 }, { bottomColor }},
        dst::gfx::VertexPositionTexcoordColor {{ -w, -h,  w }, { 0, 0 }, { bottomColor }},
        // Right
        dst::gfx::VertexPositionTexcoordColor {{  w,  h,  d }, { 0, 0 }, { topColor }},
        dst::gfx::VertexPositionTexcoordColor {{  w,  h, -d }, { 0, 0 }, { topColor }},
        dst::gfx::VertexPositionTexcoordColor {{  w, -h, -d }, { 0, 0 }, { bottomColor}},
        dst::gfx::VertexPositionTexcoordColor {{  w, -h,  d }, { 0, 0 }, { bottomColor}},
        // Back
        dst::gfx::VertexPositionTexcoordColor {{  w,  h, -d }, { 0, 0 }, { topColor }},
        dst::gfx::VertexPositionTexcoordColor {{ -w,  h, -d }, { 0, 0 }, { topColor }},
        dst::gfx::VertexPositionTexcoordColor {{ -w, -h, -d }, { 0, 0 }, { bottomColor }},
        dst::gfx::VertexPositionTexcoordColor {{  w, -h, -d }, { 0, 0 }, { bottomColor }},
        // Bottom
        dst::gfx::VertexPositionTexcoordColor {{ -w, -h,  d }, { 0, 0 }, { bottomColor }},
        dst::gfx::VertexPositionTexcoordColor {{  w, -h,  d }, { 0, 0 }, { bottomColor }},
        dst::gfx::VertexPositionTexcoordColor {{  w, -h, -d }, { 0, 0 }, { bottomColor }},
        dst::gfx::VertexPositionTexcoordColor {{ -w, -h, -d }, { 0, 0 }, { bottomColor }},
    };
    size_t index_i = 0;
    size_t vertex_i = 0;
    constexpr size_t FaceCount = 6;
    constexpr size_t IndicesPerFace = 6;
    std::array<uint16_t, IndicesPerFace * FaceCount> indices;
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

int main(int, const char*[])
{
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        // Create a gvk::Context.  This will initialize a VkInstance and VkDevice.
        gvk::Context gvkContext;
        gvk_result(dst_sample_create_gvk_context("dynamic-static - Shape Shooter", &gvkContext));
        auto gvkDevice = gvkContext.get_devices()[0];
        auto gvkQueue = gvk::get_queue_family(gvkDevice, 0).queues[0];

        // Create a gvk::system::Surface.  This is used to control a system window.
        auto systemSurfaceCreateInfo = gvk::get_default<gvk::system::Surface::CreateInfo>();
        systemSurfaceCreateInfo.pTitle = gvkContext.get_instance().get<VkInstanceCreateInfo>().pApplicationInfo->pApplicationName;
        systemSurfaceCreateInfo.extent = { 1280, 720 };
        gvk::system::Surface systemSurface;
        auto success = gvk::system::Surface::create(&systemSurfaceCreateInfo, &systemSurface);
        (void)success;
        assert(success);

        // Create a gvk::WsiManager.  This is used to manage a connection between the
        //  Vulkan context and the system window.
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
        gvk_result(gvk::WsiManager::create(gvkDevice, &wsiManagerCreateInfo, nullptr, &wsiManager));

        // Create a gvk::gui::Renderer
        gvk::gui::Renderer guiRenderer;
        gvk_result(gvk::gui::Renderer::create(
            gvkContext.get_devices()[0],
            gvk::get_queue_family(gvkContext.get_devices()[0], 0).queues[0],
            gvkContext.get_command_buffers()[0],
            wsiManager.get_render_pass(),
            nullptr,
            &guiRenderer
        ));

        // Create a gvk::RenderTarget.  We're going to want to be able to render to
        //  this gvk::RenderTarget and the gvk::WsiManager gvk::RenderTarget objects
        //  using the same gvk::Pipeline objects so the gvk::RenderPass objects need to
        //  be compatible...
        DstSampleRenderTargetCreateInfo renderTargetCreateInfo{ };
        renderTargetCreateInfo.extent = { 1024, 1024 };
        renderTargetCreateInfo.sampleCount = wsiManager.get_sample_count();
        renderTargetCreateInfo.colorFormat = wsiManager.get_color_format();
        renderTargetCreateInfo.depthFormat = wsiManager.get_depth_format();
        gvk::RenderTarget renderTarget;
        gvk_result(dst_sample_create_render_target(gvkContext, renderTargetCreateInfo, &renderTarget));

        // Create the gvk::Sampler that we'll use when we bind the gvk::RenderTarget
        //  color attachment as a shader resource...
        gvk::Sampler sampler;
        gvk_result(gvk::Sampler::create(gvkContext.get_devices()[0], &gvk::get_default<VkSamplerCreateInfo>(), nullptr, &sampler));

        // Create resources for our cube object...this includes a gvk::Mesh, a uniform
        //  gvk::Buffer, a gvk::math::Transform, and a gvk::Pipeline...
        gvk::Mesh cubeMesh;
        gvk_result(create_mesh(gvkContext, { 1, 1, 1 }, gvk::math::Color::Black, gvk::math::Color::White, &cubeMesh));
        gvk::Buffer cubeUniformBuffer;
        gvk_result(dst_sample_create_uniform_buffer<ObjectUniforms>(gvkContext.get_devices()[0], &cubeUniformBuffer));
        gvk::math::Transform cubeTransform;
        gvk::spirv::ShaderInfo vertexShaderInfo{ };
        vertexShaderInfo.language = gvk::spirv::ShadingLanguage::Glsl;
        vertexShaderInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertexShaderInfo.lineOffset = __LINE__;
        vertexShaderInfo.source = R"(
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
            } object;

            layout(location = 0) in vec3 vsPosition;
            layout(location = 1) in vec2 vsTexCoord;
            layout(location = 2) in vec4 vsColor;
            layout(location = 0) out vec2 fsTexCoord;
            layout(location = 1) out vec4 fsColor;

            out gl_PerVertex
            {
                vec4 gl_Position;
            };

            void main()
            {
                gl_Position = camera.projection * camera.view * object.world * vec4(vsPosition, 1);
                fsTexCoord = vsTexCoord;
                fsColor = vsColor;
            }
        )";
        gvk::spirv::ShaderInfo fragmentShaderInfo{ };
        fragmentShaderInfo.language = gvk::spirv::ShadingLanguage::Glsl;
        fragmentShaderInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragmentShaderInfo.lineOffset = __LINE__;
        fragmentShaderInfo.source = R"(
            #version 450

            layout(location = 0) in vec2 fsTexCoord;
            layout(location = 1) in vec4 fsColor;
            layout(location = 0) out vec4 fragColor;

            void main()
            {
                fragColor = fsColor;
            }
        )";
        gvk::Pipeline cubePipeline;
        gvk_result(dst_sample_create_pipeline<dst::gfx::VertexPositionTexcoordColor>(
            renderTarget.get_render_pass(),
            VK_CULL_MODE_NONE,
            VK_POLYGON_MODE_FILL,
            vertexShaderInfo,
            fragmentShaderInfo,
            &cubePipeline
        ));

        // Create resources for our floor object...this includes a gvk::Mesh, a uniform
        //  gvk::Buffer, a gvk::math::Transform, and a gvk::Pipeline...
        gvk::Mesh floorMesh;
        gvk_result(create_mesh(gvkContext, { 6, 0, 6 }, gvk::math::Color::White, gvk::math::Color::SlateGray, &floorMesh));
        gvk::Buffer floorUniformBuffer;
        gvk_result(dst_sample_create_uniform_buffer<ObjectUniforms>(gvkContext.get_devices()[0], &floorUniformBuffer));
        gvk::math::Transform floorTransform;
        vertexShaderInfo.language = gvk::spirv::ShadingLanguage::Glsl;
        vertexShaderInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertexShaderInfo.lineOffset = __LINE__;
        vertexShaderInfo.source = R"(
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
            } object;

            layout(location = 0) in vec3 vsPosition;
            layout(location = 1) in vec2 vsTexcoord;
            layout(location = 2) in vec4 vsColor;
            layout(location = 0) out vec4 fsPosition;
            layout(location = 1) out vec2 fsTexcoord;
            layout(location = 2) out vec4 fsColor;

            out gl_PerVertex
            {
                vec4 gl_Position;
            };
            
            void main()
            {
                fsPosition = camera.projection * camera.view * object.world * vec4(vsPosition, 1);
                gl_Position = fsPosition;
                fsTexcoord = vsTexcoord;
                fsColor = vsColor;
            }
        )";
        fragmentShaderInfo.language = gvk::spirv::ShadingLanguage::Glsl;
        fragmentShaderInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragmentShaderInfo.lineOffset = __LINE__;
        fragmentShaderInfo.source = R"(
            #version 450

            layout(set = 1, binding = 1) uniform sampler2D reflectionImage;
            layout(location = 0) in vec4 fsPosition;
            layout(location = 1) in vec2 fsTexcoord;
            layout(location = 2) in vec4 fsColor;
            layout(location = 0) out vec4 fragColor;

            void main()
            {
                // Calculate surface color
                vec4 surfaceColor;
                surfaceColor.rb = fsTexcoord;
                surfaceColor.g = dot(surfaceColor.rb, vec2(0.5));

                // Calculate reflection texcoord
                vec2 reflectionTexcoord = fsPosition.xy * (1.0 / fsPosition.w);
                reflectionTexcoord += vec2(1, 1);
                reflectionTexcoord *= 0.5;

                // Calculate reflection color
                vec4 reflectionColor = texture(reflectionImage, reflectionTexcoord);
                reflectionColor.a *= 0.34;
                reflectionColor.rgb *= reflectionColor.a;
                fragColor.rgb = surfaceColor.rgb + reflectionColor.rgb;
                fragColor.a = 1;
            }
        )";
        gvk::Pipeline floorPipeline;
        gvk_result(dst_sample_create_pipeline<dst::gfx::VertexPositionTexcoordColor>(
            renderTarget.get_render_pass(),
            VK_CULL_MODE_BACK_BIT,
            VK_POLYGON_MODE_FILL,
            vertexShaderInfo,
            fragmentShaderInfo,
            &floorPipeline
        ));

        // Create camera uniform gvk::Buffer objects...
        gvk::Buffer cameraUniformBuffer;
        gvk::Buffer reflectionCameraUniformBuffer;
        gvk_result(dst_sample_create_uniform_buffer<CameraUniforms>(gvkContext.get_devices()[0], &cameraUniformBuffer));
        gvk_result(dst_sample_create_uniform_buffer<CameraUniforms>(gvkContext.get_devices()[0], &reflectionCameraUniformBuffer));

        // Allocate gvk::DescriptorSet objects...because both gvk::Pipeline objects
        //  have individual camera and object uniforms we expect 2 gvk::DescriptorSet
        //  objects for each, with the camera gvk::DescriptorSet at index 0 and the
        //  object gvk::DescriptorSet at index 1 as defined in the vertex shader GLSL.
        //  The gvk::Pipeline that will be used to draw the floor object uses the
        //  gvk::RenderTarget as a texture to create the reflection of the cube object.
        //  The gvk::ImageView for the gvk::RenderTarget color attachment will be at
        //  gvk::DescriptorSet 1 and binding index 1...
        std::vector<gvk::DescriptorSet> descriptorSets;
        gvk_result(dst_sample_allocate_descriptor_sets(cubePipeline, descriptorSets));
        assert(descriptorSets.size() == 2);
        auto reflectionCameraDescriptorSet = descriptorSets[0];
        auto cubeDescriptorSet = descriptorSets[1];
        gvk_result(dst_sample_allocate_descriptor_sets(floorPipeline, descriptorSets));
        assert(descriptorSets.size() == 2);
        auto cameraDescriptorSet = descriptorSets[0];
        auto floorDescriptorSet = descriptorSets[1];

        // Prepare descriptors...
        auto cubeUniformBufferDescriptorInfo = gvk::get_default<VkDescriptorBufferInfo>();
        cubeUniformBufferDescriptorInfo.buffer = cubeUniformBuffer;
        auto floorUniformBufferDescriptorInfo = gvk::get_default<VkDescriptorBufferInfo>();
        floorUniformBufferDescriptorInfo.buffer = floorUniformBuffer;
        auto cameraUniformBufferDescriptorInfo = gvk::get_default<VkDescriptorBufferInfo>();
        cameraUniformBufferDescriptorInfo.buffer = cameraUniformBuffer;
        auto reflectionCameraUniformBufferDescriptorInfo = gvk::get_default<VkDescriptorBufferInfo>();
        reflectionCameraUniformBufferDescriptorInfo.buffer = reflectionCameraUniformBuffer;

        // For the VkDescriptorImageInfo we'll use the gvk::RenderTarget object's
        //  color attachment.  If the gvk::RenderTarget has multisample anti aliasing
        //  enabled, the MSAA attachment will be at index 0 and the resolve attachment
        //  will be the gvk::ImageView at index 1...
        auto colorAttachmentIndex = VK_SAMPLE_COUNT_1_BIT < renderTargetCreateInfo.sampleCount ? 1 : 0;
        assert(!renderTarget.get_framebuffer().get<gvk::ImageViews>().empty());
        auto renderTargetColorAttachmentDescriptorInfo = gvk::get_default<VkDescriptorImageInfo>();
        renderTargetColorAttachmentDescriptorInfo.sampler = sampler;
        renderTargetColorAttachmentDescriptorInfo.imageView = renderTarget.get_framebuffer().get<gvk::ImageViews>()[colorAttachmentIndex];
        renderTargetColorAttachmentDescriptorInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        // Write the descriptors...
        std::array<VkWriteDescriptorSet, 5> writeDescriptorSets{
            VkWriteDescriptorSet {
                /* .sType            = */ gvk::get_stype<VkWriteDescriptorSet>(),
                /* .pNext            = */ nullptr,
                /* .dstSet           = */ cubeDescriptorSet,
                /* .dstBinding       = */ 0,
                /* .dstArrayElement  = */ 0,
                /* .descriptorCount  = */ 1,
                /* .descriptorType   = */ VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                /* .pImageInfo       = */ nullptr,
                /* .pBufferInfo      = */ &cubeUniformBufferDescriptorInfo,
                /* .pTexelBufferView = */ nullptr,
            },
            VkWriteDescriptorSet {
                /* .sType            = */ gvk::get_stype<VkWriteDescriptorSet>(),
                /* .pNext            = */ nullptr,
                /* .dstSet           = */ floorDescriptorSet,
                /* .dstBinding       = */ 0,
                /* .dstArrayElement  = */ 0,
                /* .descriptorCount  = */ 1,
                /* .descriptorType   = */ VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                /* .pImageInfo       = */ nullptr,
                /* .pBufferInfo      = */ &floorUniformBufferDescriptorInfo,
                /* .pTexelBufferView = */ nullptr,
            },
            VkWriteDescriptorSet {
                /* .sType            = */ gvk::get_stype<VkWriteDescriptorSet>(),
                /* .pNext            = */ nullptr,
                /* .dstSet           = */ floorDescriptorSet,
                /* .dstBinding       = */ 1,
                /* .dstArrayElement  = */ 0,
                /* .descriptorCount  = */ 1,
                /* .descriptorType   = */ VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                /* .pImageInfo       = */ &renderTargetColorAttachmentDescriptorInfo,
                /* .pBufferInfo      = */ nullptr,
                /* .pTexelBufferView = */ nullptr,
            },
            VkWriteDescriptorSet {
                /* .sType            = */ gvk::get_stype<VkWriteDescriptorSet>(),
                /* .pNext            = */ nullptr,
                /* .dstSet           = */ cameraDescriptorSet,
                /* .dstBinding       = */ 0,
                /* .dstArrayElement  = */ 0,
                /* .descriptorCount  = */ 1,
                /* .descriptorType   = */ VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                /* .pImageInfo       = */ nullptr,
                /* .pBufferInfo      = */ &cameraUniformBufferDescriptorInfo,
                /* .pTexelBufferView = */ nullptr,
            },
            VkWriteDescriptorSet {
                /* .sType            = */ gvk::get_stype<VkWriteDescriptorSet>(),
                /* .pNext            = */ nullptr,
                /* .dstSet           = */ reflectionCameraDescriptorSet,
                /* .dstBinding       = */ 0,
                /* .dstArrayElement  = */ 0,
                /* .descriptorCount  = */ 1,
                /* .descriptorType   = */ VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                /* .pImageInfo       = */ nullptr,
                /* .pBufferInfo      = */ &reflectionCameraUniformBufferDescriptorInfo,
                /* .pTexelBufferView = */ nullptr,
            },
        };
        vkUpdateDescriptorSets(gvkContext.get_devices()[0], (uint32_t)writeDescriptorSets.size(), writeDescriptorSets.data(), 0, nullptr);

        // In addition to the DescriptorSets used for the 3D scene, we'll need one more
        //  to provide ImGui with the render target for display in the gui
        std::vector<gvk::DescriptorSet> guiDescriptorSets;
        gvk_result(dst_sample_allocate_descriptor_sets(guiRenderer.get_pipeline(), guiDescriptorSets));
        assert(guiDescriptorSets.size() == 1);
        auto writeDescriptorSet = gvk::get_default<VkWriteDescriptorSet>();
        writeDescriptorSet.dstSet = guiDescriptorSets[0];
        writeDescriptorSet.descriptorCount = 1;
        writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writeDescriptorSet.pImageInfo = &renderTargetColorAttachmentDescriptorInfo;
        vkUpdateDescriptorSets(gvkContext.get_devices()[0], 1, &writeDescriptorSet, 0, nullptr);

        // These variables will be controlled via gui widgets
        bool showGui = true;
        float anchor = 1.5f;
        float amplitude = 0.5f;
        float frequency = 3;
        float yRotation = 90.0f;
        float zRotation = 45.0f;
        float guiImageScale = 0.125f;
        (void)guiImageScale;

        gvk::math::Camera camera;
        camera.transform.translation = { 0, 2, -7 };
        gvk::math::FreeCameraController cameraController;
        cameraController.set_camera(&camera);

        ///////////////////////////////////////////////////////////////////////////////
        // TextMesh
        std::shared_ptr<dst::text::Font> spFont;
        // dst::text::Font::create("C:\\Windows\\Fonts\\georgia.ttf", nullptr, 256, &spFont);
        dst::text::Font::create("C:\\Windows\\Fonts\\georgia.ttf", nullptr, 64, &spFont);
        dst::gfx::Renderer<dst::text::Font> fontRenderer;
        dst::gfx::Renderer<dst::text::Font>::create(gvkContext, wsiManager.get_render_pass(), *spFont, &fontRenderer);

        dst::text::Mesh textMesh;
        textMesh.set_font(spFont);
        textMesh.set_text("The quick brown fox");
        dst::gfx::Renderer<dst::text::Mesh>* pTextMeshRenderer = nullptr;
        textMesh.create_renderer<dst::gfx::Renderer<dst::text::Mesh>>(
            [&](const auto& /*textMesh*/, auto& renderer)
            {
                pTextMeshRenderer = &renderer;
                return dst::gfx::Renderer<dst::text::Mesh>::create(gvkContext.get_devices()[0], textMesh, fontRenderer, &renderer);
            }
        );
        pTextMeshRenderer->transform.scale = glm::vec3(0.1f);
        ///////////////////////////////////////////////////////////////////////////////

        gvk::system::Clock clock;
        while (
            !(systemSurface.get_input().keyboard.down(gvk::system::Key::Escape)) &&
            !(systemSurface.get_status() & gvk::system::Surface::CloseRequested)) {
            gvk::system::Surface::update();
            clock.update();

            // Update the gvk::math::FreeCameraController...
            auto deltaTime = clock.elapsed<gvk::system::Seconds<float>>();
            const auto& input = systemSurface.get_input();

            // Toggle the gui display with [`]
            if (input.keyboard.pressed(gvk::system::Key::OEM_Tilde)) {
                showGui = !showGui;
            }

            // When ImGui wants mouse/keyboard input, input should be ignored by the scene
            if (!ImGui::GetIO().WantCaptureMouse && !ImGui::GetIO().WantCaptureKeyboard) {
                gvk::math::FreeCameraController::UpdateInfo cameraControllerUpdateInfo {
                    /* .deltaTime           = */ deltaTime,
                    /* .moveUp              = */ input.keyboard.down(gvk::system::Key::Q),
                    /* .moveDown            = */ input.keyboard.down(gvk::system::Key::E),
                    /* .moveLeft            = */ input.keyboard.down(gvk::system::Key::A),
                    /* .moveRight           = */ input.keyboard.down(gvk::system::Key::D),
                    /* .moveForward         = */ input.keyboard.down(gvk::system::Key::W),
                    /* .moveBackward        = */ input.keyboard.down(gvk::system::Key::S),
                    /* .moveSpeedMultiplier = */ input.keyboard.down(gvk::system::Key::LeftShift) ? 2.0f : 1.0f,
                    /* .lookDelta           = */ { input.mouse.position.delta()[0], input.mouse.position.delta()[1] },
                    /* .fieldOfViewDelta    = */ input.mouse.scroll.delta()[1],
                };
                cameraController.lookEnabled = input.mouse.buttons.down(gvk::system::Mouse::Button::Left);
                if (cameraController.lookEnabled) {
                    systemSurface.set_cursor_mode(gvk::system::Surface::CursorMode::Hidden);
                } else {
                    systemSurface.set_cursor_mode(gvk::system::Surface::CursorMode::Visible);
                }
                if (input.mouse.buttons.pressed(gvk::system::Mouse::Button::Right)) {
                    camera.fieldOfView = 60.0f;
                }
                cameraController.update(cameraControllerUpdateInfo);
            }

            // Update the floating cube object's gvk::math::Transform...
            cubeTransform.translation.y = anchor + amplitude * glm::sin(frequency * clock.total<gvk::system::Seconds<float>>());
            auto cubeRotationY = glm::angleAxis(glm::radians(yRotation * deltaTime), glm::vec3 { 0, 1, 0 });
            auto cubeRotationZ = glm::angleAxis(glm::radians(zRotation * deltaTime), glm::vec3 { 0, 0, 1 });
            cubeTransform.rotation = glm::normalize(cubeRotationY * cubeTransform.rotation * cubeRotationZ);

            // Uddate the gvk::math::Camera uniform data...
            CameraUniforms cameraUbo { };
            cameraUbo.view = camera.view();
            cameraUbo.projection = camera.projection();
            VmaAllocationInfo allocationInfo{ };
            vmaGetAllocationInfo(gvkContext.get_devices()[0].get<VmaAllocator>(), cameraUniformBuffer.get<VmaAllocation>(), &allocationInfo);
            assert(allocationInfo.pMappedData);
            memcpy(allocationInfo.pMappedData, &cameraUbo, sizeof(CameraUniforms));

            // Setup the reflection vk::math::Camera uniforms by scaling the view by -1 on
            //  the y axis then update the reflection gvk::math::Camera uniform data...
            cameraUbo.view = cameraUbo.view * glm::scale(glm::vec3 { 1, -1, 1 });
            vmaGetAllocationInfo(gvkContext.get_devices()[0].get<VmaAllocator>(), reflectionCameraUniformBuffer.get<VmaAllocation>(), &allocationInfo);
            assert(allocationInfo.pMappedData);
            memcpy(allocationInfo.pMappedData, &cameraUbo, sizeof(CameraUniforms));

            // Update the cube uniform data...
            ObjectUniforms cubeUbo{ };
            cubeUbo.world = cubeTransform.world_from_local();
            vmaGetAllocationInfo(gvkContext.get_devices()[0].get<VmaAllocator>(), cubeUniformBuffer.get<VmaAllocation>(), &allocationInfo);
            assert(allocationInfo.pMappedData);
            memcpy(allocationInfo.pMappedData, &cubeUbo, sizeof(ObjectUniforms));

            // Update the floor uniform data...
            ObjectUniforms floorUbo{ };
            floorUbo.world = floorTransform.world_from_local();
            vmaGetAllocationInfo(gvkContext.get_devices()[0].get<VmaAllocator>(), floorUniformBuffer.get<VmaAllocation>(), &allocationInfo);
            assert(allocationInfo.pMappedData);
            memcpy(allocationInfo.pMappedData, &floorUbo, sizeof(ObjectUniforms));


            ///////////////////////////////////////////////////////////////////////////////
            // TextMesh
#if 0
            auto textMeshRotationY = glm::angleAxis(glm::radians(yRotation * deltaTime), glm::vec3{ 0, 1, 0 });
            pTextMeshRenderer->transform.rotation = glm::normalize(textMeshRotationY * pTextMeshRenderer->transform.rotation);
#else
            pTextMeshRenderer->transform.rotation = glm::angleAxis(glm::radians(180.0f), glm::vec3 { 0, 1, 0 });
#endif
            textMesh.update(deltaTime);
            ///////////////////////////////////////////////////////////////////////////////

            wsiManager.update();
            auto swapchain = wsiManager.get_swapchain();
            if (swapchain) {
                uint32_t imageIndex = 0;
                auto vkResult = wsiManager.acquire_next_image(UINT64_MAX, VK_NULL_HANDLE, &imageIndex);
                gvk_result((vkResult == VK_SUCCESS || vkResult == VK_SUBOPTIMAL_KHR) ? VK_SUCCESS : vkResult);

                auto extent = wsiManager.get_swapchain().get<VkSwapchainCreateInfoKHR>().imageExtent;
                camera.set_aspect_ratio(extent.width, extent.height);

                // Get VkFences from the WsiManager.  The gvk::gui::Renderer will wait on these
                //  VkFences to ensure that it doesn't destroy any resources that are still in
                //  use by the WsiManager
                const auto& vkFences = wsiManager.get_vk_fences();

                // If the gvk::gui::Renderer is enabled, update values based on gui interaction
                if (showGui) {
                    // Update the gvk::system::Surface::CursorMode mode based on gui interaction
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

                    // Prepare a gvk::gui::Renderer::BeginInfo
                    const auto& textStream = systemSurface.get_text_stream();
                    auto guiRendererBeginInfo = gvk::get_default<gvk::gui::Renderer::BeginInfo>();
                    guiRendererBeginInfo.deltaTime = deltaTime;
                    guiRendererBeginInfo.extent = { (float)extent.width, (float)extent.height };
                    guiRendererBeginInfo.pInput = &input;
                    guiRendererBeginInfo.textStreamCodePointCount = (uint32_t)textStream.size();
                    guiRendererBeginInfo.pTextStreamCodePoints = !textStream.empty() ? textStream.data() : nullptr;

                    // Call guiRenderer.begin_gui().  Note that all ImGui widgets must be handled
                    //  between calls to begin_gui()/end_gui()
                    guiRenderer.begin_gui(guiRendererBeginInfo);
#if 0
                    ImGui::ShowDemoWindow();
                    ImGui::DragFloat("anchor", &anchor, 0.01f);
                    ImGui::DragFloat("amplitude", &amplitude, 0.1f);
                    ImGui::DragFloat("frequency", &frequency, 0.1f);
                    ImGui::DragFloat("yRotation", &yRotation, 0.1f);
                    ImGui::DragFloat("zRotation", &zRotation, 0.1f);
                    ImGui::DragFloat("guiImageScale", &guiImageScale, 0.005f, 0.01f, 0.5f);
                    ImVec2 guiImageExtent { renderTargetCreateInfo.extent.width * guiImageScale, renderTargetCreateInfo.extent.height * guiImageScale };
                    ImGui::Image(guiDescriptorSets[0], guiImageExtent);
#else
                    float textScale = pTextMeshRenderer->transform.scale.x;
                    if (ImGui::DragFloat("Text Scale", &textScale)) {
                        pTextMeshRenderer->transform.scale = glm::vec3(textScale);
                    }
#endif
                    guiRenderer.end_gui((uint32_t)vkFences.size(), !vkFences.empty() ? vkFences.data() : nullptr);
                }

                const auto& device = gvkContext.get_devices()[0];
                gvk_result(vkWaitForFences(device, 1, &vkFences[imageIndex], VK_TRUE, UINT64_MAX));
                gvk_result(vkResetFences(device, 1, &vkFences[imageIndex]));

                const auto& commandBuffer = wsiManager.get_command_buffers()[imageIndex];
                gvk_result(vkBeginCommandBuffer(commandBuffer, &gvk::get_default<VkCommandBufferBeginInfo>()));

                // Begin a gvk::RenderPass with our gvk::RenderTarget...
                auto renderPassBeginInfo = renderTarget.get_render_pass_begin_info();
                vkCmdBeginRenderPass(commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
                {
                    VkRect2D scissor { { }, renderPassBeginInfo.renderArea.extent };
                    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
                    VkViewport viewport { 0, 0, (float)scissor.extent.width, (float)scissor.extent.height, 0, 1 };
                    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

                    // Bind cube gvk::Pipeline, reflection gvk::math::Camera uniform gvk::Buffer,
                    //  and the cube uniform gvk::Buffer, then render the cube gvk::Mesh.  This
                    //  will draw the cube into the gvk::RenderTarget with the reflection view
                    //  matrix...in the next gvk::RenderPass, we'll use this gvk::RenderTarget as
                    //  the texture for the floor to create the illusion of a reflection on the
                    //  floor...
                    auto pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
                    vkCmdBindPipeline(commandBuffer, pipelineBindPoint, cubePipeline);
                    vkCmdBindDescriptorSets(commandBuffer, pipelineBindPoint, cubePipeline.get<gvk::PipelineLayout>(), 0, 1, &(const VkDescriptorSet&)reflectionCameraDescriptorSet, 0, nullptr);
                    vkCmdBindDescriptorSets(commandBuffer, pipelineBindPoint, cubePipeline.get<gvk::PipelineLayout>(), 1, 1, &(const VkDescriptorSet&)cubeDescriptorSet, 0, nullptr);
                    cubeMesh.record_cmds(commandBuffer);
                }
                vkCmdEndRenderPass(commandBuffer);

                // Begin the gvk::RenderPass that renders into the gvk::WsiManager...
                renderPassBeginInfo = wsiManager.get_render_targets()[imageIndex].get_render_pass_begin_info();
                vkCmdBeginRenderPass(commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
                {
                    VkRect2D scissor { { }, renderPassBeginInfo.renderArea.extent };
                    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
                    VkViewport viewport { 0, 0, (float)scissor.extent.width, (float)scissor.extent.height, 0, 1 };
                    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

                    // Bind the gvk::math::Camera uniform gvk::Buffer and the floor resources then
                    //  issue a draw call for the floor.  Then bind the floating cube resources...
                    //  we can leave the gvk::math::Camera uniform gvk::Buffer bound and update the
                    //  gvk::Pipeline and gvk::DescriptorSet at index 1 without distrubing the
                    //  gvk::DescriptorSet at index 0...then issue a draw call for the floating
                    //  cube...
                    auto pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
                    vkCmdBindPipeline(commandBuffer, pipelineBindPoint, floorPipeline);
                    vkCmdBindDescriptorSets(commandBuffer, pipelineBindPoint, floorPipeline.get<gvk::PipelineLayout>(), 0, 1, &(const VkDescriptorSet&)cameraDescriptorSet, 0, nullptr);
                    vkCmdBindDescriptorSets(commandBuffer, pipelineBindPoint, floorPipeline.get<gvk::PipelineLayout>(), 1, 1, &(const VkDescriptorSet&)floorDescriptorSet, 0, nullptr);
                    floorMesh.record_cmds(commandBuffer);
                    vkCmdBindPipeline(commandBuffer, pipelineBindPoint, cubePipeline);
                    vkCmdBindDescriptorSets(commandBuffer, pipelineBindPoint, cubePipeline.get<gvk::PipelineLayout>(), 1, 1, &(const VkDescriptorSet&)cubeDescriptorSet, 0, nullptr);
                    cubeMesh.record_cmds(commandBuffer);

                    ///////////////////////////////////////////////////////////////////////////////
                    // TextMesh
                    const auto& fontPipeline = fontRenderer.get_pipeline();
                    const auto& fontDescriptorSet = fontRenderer.get_descriptor_set();
                    vkCmdBindPipeline(commandBuffer, pipelineBindPoint, fontPipeline);
                    vkCmdBindDescriptorSets(commandBuffer, pipelineBindPoint, fontPipeline.get<gvk::PipelineLayout>(), 1, 1, &(const VkDescriptorSet&)fontDescriptorSet, 0, nullptr);
                    pTextMeshRenderer->record_draw_cmds(commandBuffer, fontRenderer);
                    ///////////////////////////////////////////////////////////////////////////////
                }

                // If the gvk::gui::Renderer is enabled, record cmds to render it
                if (showGui) {
                    guiRenderer.record_cmds(commandBuffer);
                }

                vkCmdEndRenderPass(commandBuffer);

                // Ensure the gvk::RenderTarget attachments are transitioned back to the
                //  VkImageLayout expected when the gvk::RenderPass is next executed...the
                //  VkImageMemoryBarrier objects provided by gvk::RenderTarget do not
                //  account for layout transitions that occur outside of the associated
                //  gvk::RenderPass, those must be handled by your application...
                auto attachmentCount = renderTarget.get_render_pass().get<VkRenderPassCreateInfo2>().attachmentCount;
                for (size_t attachment_i = 0; attachment_i < attachmentCount; ++attachment_i) {
                    auto imageMemoryBarrier = renderTarget.get_image_memory_barrier((uint32_t)attachment_i);
                    if (imageMemoryBarrier.oldLayout != imageMemoryBarrier.newLayout) {
                        vkCmdPipelineBarrier(
                            commandBuffer,
                            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
                            0,
                            0, nullptr,
                            0, nullptr,
                            1, &imageMemoryBarrier
                        );
                    }
                }

                gvk_result(vkEndCommandBuffer(commandBuffer));

                const auto& queue = gvk::get_queue_family(device, 0).queues[0];
                auto submitInfo = wsiManager.get_submit_info(imageIndex);
                gvk_result(vkQueueSubmit(queue, 1, &submitInfo, vkFences[imageIndex]));

                auto presentInfo = wsiManager.get_present_info(&imageIndex);
                vkResult = vkQueuePresentKHR(gvk::get_queue_family(gvkContext.get_devices()[0], 0).queues[0], &presentInfo);
                gvk_result((vkResult == VK_SUCCESS || vkResult == VK_SUBOPTIMAL_KHR) ? VK_SUCCESS : vkResult);
            }
        }
        gvk_result(vkDeviceWaitIdle(gvkContext.get_devices()[0]));
    } gvk_result_scope_end;
    if (gvkResult) {
        std::cerr << gvk::to_string(gvkResult) << std::endl;
    }
    return (int)gvkResult;
}







#if 0

struct CameraUniforms
{
    glm::mat4 view { };
    glm::mat4 projection { };
};

struct TextMeshUniforms
{
    glm::mat4 world { };
};

#include <map>

void load_image(const gvk::Context& context, const std::string& filePath, gvk::ImageView* pImageView)
{
    assert(pImageView);
    pImageView->reset();

    int width = 0;
    int height = 0;
    int channels = 0;
    stbi_uc* pData = stbi_load(filePath.c_str(), &width, &height, &channels, 4);
    assert(pData);

    auto imageCreateInfo = gvk::get_default<VkImageCreateInfo>();
    imageCreateInfo.extent.width = width;
    imageCreateInfo.extent.height = height;
    imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
    imageCreateInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
    imageCreateInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageCreateInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    VmaAllocationCreateInfo allocationCreateInfo { };
    allocationCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
    gvk::Image image;
    auto vkResult = gvk::Image::create(context.get_devices()[0], &imageCreateInfo, &allocationCreateInfo, &image);
    assert(vkResult == VK_SUCCESS);

    VmaAllocationInfo allocationInfo { };
    auto vmaAllocator = context.get_devices()[0].get<VmaAllocator>();
    vmaGetAllocationInfo(vmaAllocator, image.get<VmaAllocation>(), &allocationInfo);

    // Create the staging gvk::Buffer.
    //  Because we're using VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
    //  we should only perform sequential writes like memcpy().  See VMA's
    //  documentation for more info...
    //  https://gpuopen-librariesandsdks.github.io/VulkanMemoryAllocator/html/choosing_memory_type.html
    auto bufferCreateInfo = gvk::get_default<VkBufferCreateInfo>();
    bufferCreateInfo.size = allocationInfo.size;
    bufferCreateInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    allocationCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
    allocationCreateInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
    gvk::Buffer stagingBuffer;
    vkResult = gvk::Buffer::create(context.get_devices()[0], &bufferCreateInfo, &allocationCreateInfo, &stagingBuffer);
    assert(vkResult == VK_SUCCESS);

    // Copy the image data that we got from stb into the gvk::Buffer...
    uint8_t* pStagingData = nullptr;
    vkResult = vmaMapMemory(vmaAllocator, stagingBuffer.get<VmaAllocation>(), (void**)&pStagingData);
    assert(vkResult == VK_SUCCESS);
    memcpy(pStagingData, pData, width * height * 4 * sizeof(uint8_t));
    vmaUnmapMemory(vmaAllocator, stagingBuffer.get<VmaAllocation>());

    stbi_image_free(pData);

    vkResult = gvk::execute_immediately(
        context.get_devices()[0],
        gvk::get_queue_family(context.get_devices()[0], 0).queues[0],
        context.get_command_buffers()[0],
        VK_NULL_HANDLE,
        [&](auto)
        {
            // We start by recording a VkImageMemoryBarrier that will transition the
            //  gvk::Image to VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL and make it available at
            //  VK_PIPELINE_STAGE_TRANSFER_BIT...
            auto imageMemoryBarrier = gvk::get_default<VkImageMemoryBarrier>();
            imageMemoryBarrier.srcAccessMask = 0;
            imageMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            imageMemoryBarrier.image = image;
            vkCmdPipelineBarrier(
                context.get_command_buffers()[0],
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                VK_PIPELINE_STAGE_TRANSFER_BIT,
                0,
                0, nullptr,
                0, nullptr,
                1, &imageMemoryBarrier
            );

            // Copy from the staging resource to the gvk::Image...
            auto bufferImageCopy = gvk::get_default<VkBufferImageCopy>();
            bufferImageCopy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            bufferImageCopy.imageSubresource.layerCount = 1;
            bufferImageCopy.imageExtent = imageCreateInfo.extent;
            vkCmdCopyBufferToImage(context.get_command_buffers()[0], stagingBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &bufferImageCopy);

            // Then transition the gvk::Image to VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
            //  after VK_PIPELINE_STAGE_TRANSFER_BIT and make it available at
            //  VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT since it will be sampled from during
            //  fragment shading...
            imageMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            imageMemoryBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            vkCmdPipelineBarrier(
                context.get_command_buffers()[0],
                VK_PIPELINE_STAGE_TRANSFER_BIT,
                VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                0,
                0, nullptr,
                0, nullptr,
                1, &imageMemoryBarrier
            );
        }
    );
    assert(vkResult == VK_SUCCESS);

    // Create gvk::ImageView...
    auto imageViewCreateInfo = gvk::get_default<VkImageViewCreateInfo>();
    imageViewCreateInfo.image = image;
    imageViewCreateInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    imageViewCreateInfo.format = imageCreateInfo.format;
    vkResult = gvk::ImageView::create(context.get_devices()[0], &imageViewCreateInfo, nullptr, pImageView);
    assert(vkResult == VK_SUCCESS);
}

std::map<std::string, gvk::ImageView> load_images(const gvk::Context& context)
{
    std::map<std::string, gvk::ImageView> images {
        { SHAPE_SHOOTER_CONTENT "/Art/Black Hole.png", VK_NULL_HANDLE },
        { SHAPE_SHOOTER_CONTENT "/Art/Bullet.png",     VK_NULL_HANDLE },
        { SHAPE_SHOOTER_CONTENT "/Art/Glow.png",       VK_NULL_HANDLE },
        { SHAPE_SHOOTER_CONTENT "/Art/Laser.png",      VK_NULL_HANDLE },
        { SHAPE_SHOOTER_CONTENT "/Art/Player.png",     VK_NULL_HANDLE },
        { SHAPE_SHOOTER_CONTENT "/Art/Pointer.png",    VK_NULL_HANDLE },
        { SHAPE_SHOOTER_CONTENT "/Art/Seeker.png",     VK_NULL_HANDLE },
        { SHAPE_SHOOTER_CONTENT "/Art/Wanderer.png",   VK_NULL_HANDLE },
    };
    for (auto& image : images) {
        load_image(context, image.first, &image.second);
    }
    return images;
}

namespace shape_shooter {

class GameInfo final
{
public:
};

using GameState = dst::State<const GameInfo&>;

namespace game_state {

class Attract;
class Spawning;
class Playing;
class Celebration;
class GameOver;
class Resetting;

class Attract final
    : public GameState
{
public:
    void enter(const State* pExiting) override final
    {
        (void)pExiting;
    }

    void update(State::Machine& stateMachine, const GameInfo& gameInfo) override final
    {
        (void)stateMachine;
        (void)gameInfo;
    }

    void exit(const State* pEntering) override final
    {
        (void)pEntering;
    }
};

class Spawning final
    : public GameState
{
public:
    void enter(const State* pExiting) override final
    {
        (void)pExiting;
    }

    void update(State::Machine& stateMachine, const GameInfo& gameInfo) override final
    {
        (void)stateMachine;
        (void)gameInfo;
    }

    void exit(const State* pEntering) override final
    {
        (void)pEntering;
    }
};

class Playing final
    : public GameState
{
public:
    void enter(const State* pExiting) override final
    {
        (void)pExiting;
    }

    void update(State::Machine& stateMachine, const GameInfo& gameInfo) override final
    {
        (void)stateMachine;
        (void)gameInfo;
    }

    void exit(const State* pEntering) override final
    {
        (void)pEntering;
    }
};

class Celebration final
    : public GameState
{
public:
    void enter(const State* pExiting) override final
    {
        (void)pExiting;
    }

    void update(State::Machine& stateMachine, const GameInfo& gameInfo) override final
    {
        (void)stateMachine;
        (void)gameInfo;
    }

    void exit(const State* pEntering) override final
    {
        (void)pEntering;
    }
};

class GameOver final
    : public GameState
{
public:
    void enter(const State* pExiting) override final
    {
        (void)pExiting;
    }

    void update(State::Machine& stateMachine, const GameInfo& gameInfo) override final
    {
        (void)stateMachine;
        (void)gameInfo;
    }

    void exit(const State* pEntering) override final
    {
        (void)pEntering;
    }
};

class Resetting final
    : public GameState
{
public:
    void enter(const State* pExiting) override final
    {
        (void)pExiting;
    }

    void update(State::Machine& stateMachine, const GameInfo& gameInfo) override final
    {
        (void)stateMachine;
        (void)gameInfo;
    }

    void exit(const State* pEntering) override final
    {
        (void)pEntering;
    }
};

} // namespace game_state
} // namespace shape_shooter

int main(int, const char* [])
{
    // Attract
    // Spawning
    // Playing
    // Celebration
    // GameOver
    // Resetting

    shape_shooter::GameState::Machine gameStateMachine;
    gameStateMachine.add_state<shape_shooter::game_state::Attract>();
    gameStateMachine.add_state<shape_shooter::game_state::Spawning>();
    gameStateMachine.add_state<shape_shooter::game_state::Playing>();
    gameStateMachine.add_state<shape_shooter::game_state::Celebration>();
    gameStateMachine.add_state<shape_shooter::game_state::GameOver>();
    gameStateMachine.add_state<shape_shooter::game_state::Resetting>();
    gameStateMachine.set_state<shape_shooter::game_state::Attract>();
    shape_shooter::GameInfo gameInfo { };
    gameStateMachine.update(gameInfo);

#if 0
    dst::Image<> image;
    auto& texel = image[{ 16, 32 }];
    (void)texel;
#endif

    // Create a gvk::Context.  This will initialize a VkInstance and VkDevice.
    gvk::Context gvkContext;
    gvk_result(dst_sample_create_gvk_context("dynamic-static - Shape Shooter", &gvkContext));
    auto gvkDevice = gvkContext.get_devices()[0];
    auto gvkQueue = gvk::get_queue_family(gvkDevice, 0).queues[0];

    auto images = load_images(gvkContext);
    (void)images;

#if 0
    FMOD::System* pFmodSystem = nullptr;
    auto fmodResult = FMOD::System_Create(&pFmodSystem);
    assert(fmodResult == FMOD_OK);

    int audioDriverCount = 0;
    fmodResult = pFmodSystem->getNumDrivers(&audioDriverCount);
    assert(fmodResult == FMOD_OK);
    assert(audioDriverCount);

    fmodResult = pFmodSystem->init(36, FMOD_INIT_NORMAL, NULL);
    assert(fmodResult == FMOD_OK);

    FMOD::Sound* pFmodSound = nullptr;
    fmodResult = pFmodSystem->createSound(SHAPE_SHOOTER_CONTENT "/Audio/Music.mp3", FMOD_DEFAULT, 0, &pFmodSound);
    assert(fmodResult == FMOD_OK);

    fmodResult = pFmodSound->setMode(FMOD_LOOP_NORMAL);
    assert(fmodResult == FMOD_OK);
    fmodResult = pFmodSound->setLoopCount(-1);
    assert(fmodResult == FMOD_OK);
    fmodResult = pFmodSystem->playSound(pFmodSound, nullptr, false, nullptr);
    assert(fmodResult == FMOD_OK);

    bool loop = true;
    while (loop) {
        int b = 0;
        (void)b;
    }

    fmodResult = pFmodSound->release();
    assert(fmodResult == FMOD_OK);

    fmodResult = pFmodSystem->close();
    assert(fmodResult == FMOD_OK);

    fmodResult = pFmodSystem->release();
    assert(fmodResult == FMOD_OK);
#endif

    // Create a gvk::system::Surface.  This is used to control a system window.
    auto systemSurfaceCreateInfo = gvk::get_default<gvk::system::Surface::CreateInfo>();
    systemSurfaceCreateInfo.pTitle = gvkContext.get_instance().get<VkInstanceCreateInfo>().pApplicationInfo->pApplicationName;
    systemSurfaceCreateInfo.extent = { 1280, 720 };
    gvk::system::Surface systemSurface;
    auto success = gvk::system::Surface::create(&systemSurfaceCreateInfo, &systemSurface);
    (void)success;
    assert(success);

    // Create a gvk::WsiManager.  This is used to manage a connection between the
    //  Vulkan context and the system window.
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
    gvk_result(gvk::WsiManager::create(gvkDevice, &wsiManagerCreateInfo, nullptr, &wsiManager));

    ///////////////////////////////////////////////////////////////////////////////



    std::shared_ptr<dst::text::Font> spFont;
    dst::text::Font::create("C:\\Windows\\Fonts\\georgia.ttf", nullptr, 256, &spFont);
    dst::gfx::Renderer<dst::text::Font> fontRenderer;
    dst::gfx::Renderer<dst::text::Font>::create(gvkContext, wsiManager.get_render_pass(), *spFont, &fontRenderer);

    dst::text::Mesh textMesh;
    textMesh.set_font(spFont);
    textMesh.set_text("The quick brown fox");
    dst::gfx::Renderer<dst::text::Mesh>* pTextMeshRenderer = nullptr;
    textMesh.create_renderer<dst::gfx::Renderer<dst::text::Mesh>>(
        [&](const auto& /*textMesh*/, auto& renderer)
        {
            pTextMeshRenderer = &renderer;
            return dst::gfx::Renderer<dst::text::Mesh>::create(gvkContext.get_devices()[0], textMesh, fontRenderer, &renderer);
        }
    );

    ///////////////////////////////////////////////////////////////////////////////

    gvk::math::Camera camera;
    camera.transform.translation = { 0, 2, -7 };
    gvk::math::FreeCameraController cameraController;
    cameraController.set_camera(&camera);
    gvk::Buffer cameraUniformBuffer;
    gvk_result(dst_sample_create_uniform_buffer<CameraUniforms>(gvkContext.get_devices()[0], &cameraUniformBuffer));
    std::vector<gvk::DescriptorSet> descriptorSets;
    gvk_result(dst_sample_allocate_descriptor_sets(fontRenderer.get_pipeline(), descriptorSets));
    assert(descriptorSets.size() == 3);
    auto cameraDescriptorSet = descriptorSets[0];
    auto cameraUniformBufferDescriptorInfo = gvk::get_default<VkDescriptorBufferInfo>();
    cameraUniformBufferDescriptorInfo.buffer = cameraUniformBuffer;
    auto cameraWriteDescriptorSet = gvk::get_default<VkWriteDescriptorSet>();
    cameraWriteDescriptorSet.dstSet = cameraDescriptorSet;
    cameraWriteDescriptorSet.descriptorCount = 1;
    cameraWriteDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    cameraWriteDescriptorSet.pBufferInfo = &cameraUniformBufferDescriptorInfo;
    vkUpdateDescriptorSets(gvkContext.get_devices()[0], 1, &cameraWriteDescriptorSet, 0, nullptr);

    gvk::system::Clock clock;
    while (
        !(systemSurface.get_input().keyboard.down(gvk::system::Key::Escape)) &&
        !(systemSurface.get_status() & gvk::system::Surface::CloseRequested)) {
        gvk::system::Surface::update();
        clock.update();

        auto deltaTime = clock.elapsed<gvk::system::Seconds<float>>();
        const auto& input = systemSurface.get_input();
        (void)input;

        gvk::math::FreeCameraController::UpdateInfo cameraControllerUpdateInfo{
            /* .deltaTime           = */ deltaTime,
            /* .moveUp              = */ input.keyboard.down(gvk::system::Key::Q),
            /* .moveDown            = */ input.keyboard.down(gvk::system::Key::E),
            /* .moveLeft            = */ input.keyboard.down(gvk::system::Key::A),
            /* .moveRight           = */ input.keyboard.down(gvk::system::Key::D),
            /* .moveForward         = */ input.keyboard.down(gvk::system::Key::W),
            /* .moveBackward        = */ input.keyboard.down(gvk::system::Key::S),
            /* .moveSpeedMultiplier = */ input.keyboard.down(gvk::system::Key::LeftShift) ? 2.0f : 1.0f,
            /* .lookDelta           = */ { input.mouse.position.delta()[0], input.mouse.position.delta()[1] },
            /* .fieldOfViewDelta    = */ input.mouse.scroll.delta()[1],
        };
        cameraController.lookEnabled = input.mouse.buttons.down(gvk::system::Mouse::Button::Left);
        if (cameraController.lookEnabled) {
            systemSurface.set_cursor_mode(gvk::system::Surface::CursorMode::Hidden);
        }
        else {
            systemSurface.set_cursor_mode(gvk::system::Surface::CursorMode::Visible);
        }
        if (input.mouse.buttons.pressed(gvk::system::Mouse::Button::Right)) {
            camera.fieldOfView = 60.0f;
        }
        cameraController.update(cameraControllerUpdateInfo);

        CameraUniforms cameraUbo{ };
        cameraUbo.view = camera.view();
        cameraUbo.projection = camera.projection();
        VmaAllocationInfo allocationInfo{ };
        vmaGetAllocationInfo(gvkContext.get_devices()[0].get<VmaAllocator>(), cameraUniformBuffer.get<VmaAllocation>(), &allocationInfo);
        assert(allocationInfo.pMappedData);
        memcpy(allocationInfo.pMappedData, &cameraUbo, sizeof(CameraUniforms));

        textMesh.update(deltaTime);

        // Call wsiManager.update().  This will cause WsiManager to respond to system
        //  updates for the SurfaceKHR it's managing.  This call may cause resources to
        //  be created/destroyed.  If there's a valid SwapchainKHR, render and present.
        wsiManager.update();
        auto swapchain = wsiManager.get_swapchain();
        if (swapchain) {
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
            gvk_result(vkWaitForFences(gvkDevice, 1, &vkFences[imageIndex], VK_TRUE, UINT64_MAX));
            gvk_result(vkResetFences(gvkDevice, 1, &vkFences[imageIndex]));

            // Begin CommandBuffer recording and begin a RenderPass.
            const auto& commandBuffer = wsiManager.get_command_buffers()[imageIndex];
            gvk_result(vkBeginCommandBuffer(commandBuffer, &gvk::get_default<VkCommandBufferBeginInfo>()));
            auto renderPassBeginInfo = wsiManager.get_render_targets()[imageIndex].get_render_pass_begin_info();
            vkCmdBeginRenderPass(commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

            // Set the scissor and viewport to match the renderPassBeginInfo.renderArea
            VkRect2D scissor { .extent = renderPassBeginInfo.renderArea.extent };
            vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
            VkViewport viewport { .width = (float)scissor.extent.width, .height = (float)scissor.extent.height, .minDepth = 0, .maxDepth = 1 };
            vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

            {
                // TODO :
                auto bindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
                const auto& fontPipeline = fontRenderer.get_pipeline();
                const auto& fontDescriptorSet = fontRenderer.get_descriptor_set();
                vkCmdBindPipeline(commandBuffer, bindPoint, fontPipeline);
                vkCmdBindDescriptorSets(commandBuffer, bindPoint, fontPipeline.get<gvk::PipelineLayout>(), 0, 1, &(const VkDescriptorSet&)cameraDescriptorSet, 0, nullptr);
                vkCmdBindDescriptorSets(commandBuffer, bindPoint, fontPipeline.get<gvk::PipelineLayout>(), 1, 1, &(const VkDescriptorSet&)fontDescriptorSet, 0, nullptr);
                pTextMeshRenderer->record_draw_cmds(commandBuffer, fontRenderer);
            }

            // End the RenderPass and CommandBuffer.
            vkCmdEndRenderPass(commandBuffer);
            gvk_result(vkEndCommandBuffer(commandBuffer));

            // Submit the CommandBuffer for execution on the GPU.
            auto submitInfo = wsiManager.get_submit_info(imageIndex);
            gvk_result(vkQueueSubmit(gvkQueue, 1, &submitInfo, vkFences[imageIndex]));

            // Present the SwapchainKHR Image that was drawn into.
            auto presentInfo = wsiManager.get_present_info(&imageIndex);
            vkResult = vkQueuePresentKHR(gvkQueue, &presentInfo);
            assert(vkResult == VK_SUCCESS || vkResult == VK_SUBOPTIMAL_KHR);
        }
    }

    // Wait for the GPU to be idle before allowing graphics destructors to run.
    gvk_result(vkDeviceWaitIdle(gvkContext.get_devices()[0]));

    return 0;
}

#endif
