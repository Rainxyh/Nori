#pragma once

#include <nori/common.h>
#include <thread>
#include <nori/block.h>
#include <atomic>

NORI_NAMESPACE_BEGIN

class RenderThread {

public:
    RenderThread(ImageBlock & block);
    ~RenderThread();

    void renderScene(const std::string & filename, bool singleThreaded);

    bool isBusy();
    void stopRendering();

    float getProgress();

protected:
    Scene* m_scene = nullptr;
    ImageBlock & m_block;
    std::thread m_render_thread;
    std::atomic<int> m_render_status; // 0: free, 1: busy, 2: interruption, 3: done
    std::atomic<float> m_progress;

};

NORI_NAMESPACE_END