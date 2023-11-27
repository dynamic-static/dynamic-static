
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

#pragma once

#include "shape-shooter/camera.hpp"

namespace shape_shooter {

VkResult Camera::create(const gvk::DescriptorSetLayout& descriptorSetLayout, Camera* pCamera)
{
    assert(descriptorSetLayout);
    assert(pCamera);
    pCamera->reset();
    gvk_result_scope_begin(VK_ERROR_INITIALIZATION_FAILED) {
        const auto& device = descriptorSetLayout.get<gvk::Device>();

        // TODO : Documentation
        gvk_result(dst_sample_create_uniform_buffer<Uniforms>(device, &pCamera->mUniformBuffer));

        // TODO : Documentation
        auto descriptorPoolSize = gvk::get_default<VkDescriptorPoolSize>();
        descriptorPoolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        descriptorPoolSize.descriptorCount = 1;
        auto descriptorPoolCreateInfo = gvk::get_default<VkDescriptorPoolCreateInfo>();
        descriptorPoolCreateInfo.maxSets = 1;
        descriptorPoolCreateInfo.poolSizeCount = 1;
        descriptorPoolCreateInfo.pPoolSizes = &descriptorPoolSize;
        gvk::DescriptorPool descriptorPool;
        gvk_result(gvk::DescriptorPool::create(device, &descriptorPoolCreateInfo, nullptr, &descriptorPool));

        // TODO : Documentation
        auto descriptorSetAllocateInfo = gvk::get_default<VkDescriptorSetAllocateInfo>();
        descriptorSetAllocateInfo.descriptorPool = descriptorPool;
        descriptorSetAllocateInfo.descriptorSetCount = 1;
        descriptorSetAllocateInfo.pSetLayouts = &descriptorSetLayout.get<VkDescriptorSetLayout>();
        gvk_result(gvk::DescriptorSet::allocate(device, &descriptorSetAllocateInfo, &pCamera->mDescriptorSet));

        // TODO : Documentation
        auto descriptorBufferInfo = gvk::get_default<VkDescriptorBufferInfo>();
        descriptorBufferInfo.buffer = pCamera->mUniformBuffer;

        // TODO : Documentation
        auto writeDescriptorSet = gvk::get_default<VkWriteDescriptorSet>();
        writeDescriptorSet.descriptorCount = 1;
        writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        writeDescriptorSet.pBufferInfo = &descriptorBufferInfo;
        device.get<gvk::DispatchTable>().gvkUpdateDescriptorSets(device, 1, &writeDescriptorSet, 0, nullptr);
    } gvk_result_scope_end;
    return gvkResult;
}

Camera::~Camera()
{
    reset();
}

void Camera::reset()
{
    mUniformBuffer.reset();
    mDescriptorSet.reset();
}

const gvk::DescriptorSet& Camera::get_descriptor_set() const
{
    return mDescriptorSet;
}

void Camera::update()
{

}

} // namespace shape_shooter
