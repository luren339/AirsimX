#pragma once

#include "CoreMinimal.h"
#include "Engine/TextureRenderTarget2D.h"
#include "common/WorkerThread.hpp"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/GameViewportClient.h"
#include <memory>
#include "common/Common.hpp"

class FRHICommandListImmediate;

class RenderRequest
{
public:
    struct RenderParams
    {
        USceneCaptureComponent2D* const render_component;
        UTextureRenderTarget2D* render_target;
        bool pixels_as_float;
        bool compress;

        RenderParams(USceneCaptureComponent2D* render_component_val, UTextureRenderTarget2D* render_target_val, bool pixels_as_float_val, bool compress_val)
            : render_component(render_component_val), render_target(render_target_val), pixels_as_float(pixels_as_float_val), compress(compress_val)
        {
        }
    };
    struct RenderResult
    {
        TArray<uint8> image_data_uint8;
        TArray<float> image_data_float;

        TArray<FColor> bmp;
        TArray<FFloat16Color> bmp_float;

        int width = 0;
        int height = 0;

        msr::airlib::TTimePoint time_stamp = 0;
    };

private:
    static FReadSurfaceDataFlags setupRenderResource(const FTextureRenderTargetResource* rt_resource, const RenderParams* params, RenderResult* result, FIntPoint& size);

    std::shared_ptr<RenderParams>* params_;
    std::shared_ptr<RenderResult>* results_;
    unsigned int req_size_;

    std::shared_ptr<msr::airlib::WorkerThreadSignal> wait_signal_;

    bool saved_DisableWorldRendering_ = false;
    UGameViewportClient* const game_viewport_;
    FDelegateHandle end_draw_handle_;
    std::function<void()> query_camera_pose_cb_;

public:
    RenderRequest(UGameViewportClient* game_viewport, std::function<void()>&& query_camera_pose_cb);
    ~RenderRequest();

    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(RenderRequest, STATGROUP_RenderThreadCommands);
    }

    // read pixels from render target using render thread, then compress the result into PNG
    // argument on the thread that calls this method.
    void getScreenshot(
        std::shared_ptr<RenderParams> params[], std::vector<std::shared_ptr<RenderResult>>& results, unsigned int req_size, bool use_safe_method);

    void ExecuteTask(FRHICommandListImmediate& RHICmdList);
};
