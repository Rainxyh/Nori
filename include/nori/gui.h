#pragma once

#include <nori/common.h>
#include <nanogui/screen.h>

NORI_NAMESPACE_BEGIN

class NoriScreen : public nanogui::Screen {
public:
    NoriScreen(const ImageBlock &block);
    void draw_contents() override;
private:
    const ImageBlock &m_block;
    nanogui::ref<nanogui::Shader>     m_shader;
    nanogui::ref<nanogui::Texture>    m_texture;
    nanogui::ref<nanogui::RenderPass> m_renderPass;
    float m_scale = 1.f;
};

NORI_NAMESPACE_END
