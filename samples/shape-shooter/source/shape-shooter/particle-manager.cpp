
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

#include "shape-shooter/particle-manager.hpp"
#include "shape-shooter/context.hpp"
#include "shape-shooter/utilities.hpp"

namespace shape_shooter {

void ParticleManager::resize(size_t capacity)
{
    mParticles.resize(capacity);
}

void ParticleManager::add(const Particle& particle)
{
    mParticles.push_back(particle);
}

void ParticleManager::update()
{
    const auto& context = Context::instance();
    auto playFieldHalfExtent = context.playField.extent * 0.5f;
    for (size_t i = 0; i < mParticles.count();) {
        auto& particle = mParticles[i];

#if 1
        // TODO : Documentation
        auto speed = glm::length(particle.velocity);
        particle.position += particle.velocity; // * deltaTime;
        (void)speed;
#endif

#if 1
        // TODO : Documentation
        auto alpha = std::min(std::min(particle.percentLife * 2.0f, speed), 1.0f);
        alpha *= alpha;
        particle.color.a = alpha;
#endif

        // TODO : Documentation
        if (particle.type == Particle::Type::Bullet) {
            particle.scale.x = particle.lengthMultiplier * std::min(std::min(1.0f, 0.1f * speed + 0.1f), alpha);
        } else {
            particle.scale.x = particle.lengthMultiplier * std::min(std::min(1.0f, 0.2f * speed + 0.1f), alpha);
        }

        // TODO : Documentation
        particle.orientation = std::atan2(-particle.velocity.z, particle.velocity.x); // to_angle()? get_orientation()?

#if 1
        // TODO : Documentation
        if (particle.position.x < -playFieldHalfExtent.x) {
            particle.velocity.x = std::abs(particle.velocity.x);
        } else if (playFieldHalfExtent.x < particle.position.x) {
            particle.velocity.x = -std::abs(particle.velocity.x);
        }
        if (particle.position.z < -playFieldHalfExtent.z) {
            particle.velocity.z = std::abs(particle.velocity.z);
        } else if (playFieldHalfExtent.z < particle.position.z) {
            particle.velocity.z = -std::abs(particle.velocity.z);
        }
#endif

        // TODO : Documentation
        // TODO : Blackholes

        // TODO : Documentation
        if (std::abs(particle.velocity.x) + std::abs(particle.velocity.y) < 0.00000000001f) {
            particle.velocity = { };
        } else if (particle.type == Particle::Type::Enemy) {
            particle.velocity *= 0.94f;
        } else {
            particle.velocity *= 0.96f + glm::mod(std::abs(particle.position.x), 0.04f);
        }

        // TODO : Documentation
        particle.percentLife -= 1.0f / particle.duration;
        if (particle.percentLife <= 0) {
            std::swap(particle, mParticles.back());
            mParticles.pop_back();
        } else {
            ++i;
        }
    }
}

void ParticleManager::draw() const
{
    auto& spriteRenderer = Context::instance().spriteRenderer;
    for (size_t i = 0; i < mParticles.count(); ++i) {
        auto& particle = mParticles[i];
        gvk::math::Transform transform{ };
        transform.translation = SpriteOffset + particle.position;
        transform.rotation = glm::angleAxis(particle.orientation, glm::vec3{ 0, 1, 0 }) * glm::angleAxis(glm::radians(90.0f), glm::vec3{ 1, 0, 0 });
        transform.scale *= SpriteScale * particle.scale;
        spriteRenderer.submit((uint32_t)particle.sprite, transform, particle.color);
    }
}

} // namespace shape_shooter
