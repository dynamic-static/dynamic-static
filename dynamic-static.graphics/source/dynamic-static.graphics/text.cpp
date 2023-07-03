
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

#include "dynamic-static.graphics/text.hpp"

namespace dst {
namespace gfx {

VkResult Renderer<dst::text::Font>::create(const gvk::DescriptorPool& descriptorPool, const dst::text::Font& font, Renderer<dst::text::Font>* pRenderer)
{
    assert(descriptorPool);
    assert(pRenderer);
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        const auto& device = descriptorPool.get<gvk::Device>();
        gvk_result(pRenderer->create_pipline(device, font));
        gvk_result(pRenderer->create_image_views(device, font));
        gvk_result(pRenderer->allocate_descriptor_set(device, descriptorPool));
        pRenderer->update_descriptor_set(device);
    } gvk_result_scope_end;
    return gvkResult;
}

VkResult Renderer<dst::text::Font>::create_pipline(const gvk::Device& device, const dst::text::Font& font)
{
    (void)font;
    assert(device);
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        gvk_result(VK_ERROR_INITIALIZATION_FAILED);
    } gvk_result_scope_end;
    return gvkResult;
}

VkResult Renderer<dst::text::Font>::create_image_views(const gvk::Device& device, const dst::text::Font& font)
{
    (void)font;
    assert(device);
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        gvk_result(VK_ERROR_INITIALIZATION_FAILED);
    } gvk_result_scope_end;
    return gvkResult;
}

VkResult Renderer<dst::text::Font>::allocate_descriptor_set(const gvk::Device& device, const gvk::DescriptorPool& descriptorPool)
{
    assert(device);
    assert(descriptorPool);
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        gvk_result(VK_ERROR_INITIALIZATION_FAILED);
    } gvk_result_scope_end;
    return gvkResult;
}

void Renderer<dst::text::Font>::update_descriptor_set(const gvk::Device& device)
{
    assert(device);
}

void Renderer<dst::text::Font>::record_bind_cmds(const gvk::CommandBuffer& commandBuffer)
{
    assert(commandBuffer);
    const auto& device = commandBuffer.get<gvk::Device>();
    const auto& dispatchTable = device.get<gvk::DispatchTable>();
    auto bindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    dispatchTable.gvkCmdBindPipeline(commandBuffer, bindPoint, mPipeline);
    dispatchTable.gvkCmdBindDescriptorSets(commandBuffer, bindPoint, mPipeline.get<gvk::PipelineLayout>(), 0, 1, &mDescriptorSet.get<const VkDescriptorSet&>(), 0, nullptr);
}

VkResult Renderer<dst::text::Mesh>::create(Renderer<dst::text::Mesh>* pRenderer)
{
    assert(pRenderer);
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        gvk_result(VK_ERROR_INITIALIZATION_FAILED);
    } gvk_result_scope_end;
    return gvkResult;
}

} // namespace gfx
} // namespace dst
