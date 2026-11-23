#include "RenderRequest.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Async/TaskGraphInterfaces.h"
#include "ImageUtils.h"
#include "RHICommandList.h"

#include "AirBlueprintLib.h"
#include "Async/Async.h"

RenderRequest::RenderRequest(UGameViewportClient* game_viewport, std::function<void()>&& query_camera_pose_cb)
    : params_(nullptr), results_(nullptr), req_size_(0), wait_signal_(new msr::airlib::WorkerThreadSignal), game_viewport_(game_viewport), query_camera_pose_cb_(std::move(query_camera_pose_cb))
{
}

RenderRequest::~RenderRequest()
{
}

// read pixels from render target using render thread, then compress the result into PNG
// argument on the thread that calls this method.
void RenderRequest::getScreenshot(std::shared_ptr<RenderParams> params[], std::vector<std::shared_ptr<RenderResult>>& results, unsigned int req_size, bool use_safe_method)
{
    //TODO: is below really needed?
    for (unsigned int i = 0; i < req_size; ++i) {
        results.push_back(std::make_shared<RenderResult>());

        if (!params[i]->pixels_as_float)
            results[i]->bmp.Reset();
        else
            results[i]->bmp_float.Reset();
        results[i]->time_stamp = 0;
    }

    //make sure we are not on the rendering thread
    CheckNotBlockedOnRenderThread();

    if (req_size == 0)
        return;

    if (use_safe_method) {
        UAirBlueprintLib::RunCommandOnGameThread([&]() {
            for (unsigned int i = 0; i < req_size; ++i) {
                auto rt_resource = params[i]->render_target ? params[i]->render_target->GameThread_GetRenderTargetResource() : nullptr;
                if (rt_resource == nullptr)
                    continue;
                FIntPoint img_size;
                auto flags = setupRenderResource(rt_resource, params[i].get(), results[i].get(), img_size);
                if (!params[i]->pixels_as_float)
                    rt_resource->ReadPixels(results[i]->bmp, flags);
                else
                    rt_resource->ReadFloat16Pixels(results[i]->bmp_float, flags);
                results[i]->time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
            }
        }, true);
    }
    else {
        //wait for render thread to pick up our task
        params_ = params;
        results_ = results.data();
        req_size_ = req_size;

        // Queue up the task of querying camera pose in the game thread and synchronizing render thread with camera pose
        AsyncTask(ENamedThreads::GameThread, [this]() {
            check(IsInGameThread());

            saved_DisableWorldRendering_ = game_viewport_->bDisableWorldRendering;
            game_viewport_->bDisableWorldRendering = 0;
            end_draw_handle_ = game_viewport_->OnEndDraw().AddLambda([this] {
                check(IsInGameThread());

                // capture CameraPose for this frame
                query_camera_pose_cb_();

                game_viewport_->bDisableWorldRendering = saved_DisableWorldRendering_;
                assert(end_draw_handle_.IsValid());
                // Removing the delegate destroys this callback, including its captures.
                RenderRequest* This = this;
                game_viewport_->OnEndDraw().Remove(end_draw_handle_);

                // Finish game-thread access before readback signals the caller, which may destroy this request.
                ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)
                (
                    [This](FRHICommandListImmediate& RHICmdList) {
                        This->ExecuteTask(RHICmdList);
                    });
            });

            // while we're still on GameThread, enqueue request for capture the scene!
            for (unsigned int i = 0; i < req_size_; ++i) {
                if (params_[i]->render_component != nullptr && params_[i]->render_target != nullptr)
                    params_[i]->render_component->CaptureSceneDeferred();
            }
        });

        // wait for this task to complete
        while (!wait_signal_->waitFor(5)) {
            // log a message and continue wait
            // lamda function still references a few objects for which there is no refcount.
            // Walking away will cause memory corruption, which is much more difficult to debug.
            UE_LOG(LogTemp, Warning, TEXT("Failed: timeout waiting for screenshot"));
        }
    }

    for (unsigned int i = 0; i < req_size; ++i) {
        const int64 expected_pixels = static_cast<int64>(results[i]->width) * results[i]->height;
        const int64 actual_pixels = params[i]->pixels_as_float ? results[i]->bmp_float.Num() : results[i]->bmp.Num();
        if (expected_pixels <= 0 || expected_pixels != actual_pixels) {
            results[i]->width = results[i]->height = 0;
            continue;
        }
        if (!params[i]->pixels_as_float) {
            if (results[i]->width != 0 && results[i]->height != 0) {
                results[i]->image_data_uint8.SetNumUninitialized(results[i]->width * results[i]->height * 3);
                if (params[i]->compress)
                    UAirBlueprintLib::CompressImageArray(results[i]->width, results[i]->height, results[i]->bmp, results[i]->image_data_uint8);
                else {
                    uint8* ptr = results[i]->image_data_uint8.GetData();
                    for (const auto& item : results[i]->bmp) {
                        *ptr++ = item.B;
                        *ptr++ = item.G;
                        *ptr++ = item.R;
                    }
                }
            }
        }
        else {
            results[i]->image_data_float.SetNumUninitialized(results[i]->width * results[i]->height);
            float* ptr = results[i]->image_data_float.GetData();
            for (const auto& item : results[i]->bmp_float) {
                *ptr++ = item.R.GetFloat();
            }
        }
    }
}

FReadSurfaceDataFlags RenderRequest::setupRenderResource(const FTextureRenderTargetResource* rt_resource, const RenderParams* params, RenderResult* result, FIntPoint& size)
{
    size = rt_resource->GetSizeXY();
    result->width = size.X;
    result->height = size.Y;
    FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
    flags.SetLinearToGamma(false);

    return flags;
}

void RenderRequest::ExecuteTask(FRHICommandListImmediate& RHICmdList)
{
    check(IsInRenderingThread());
    if (params_ != nullptr && req_size_ > 0) {
        for (unsigned int i = 0; i < req_size_; ++i) {
            auto rt_resource = params_[i]->render_target ? params_[i]->render_target->GetRenderTargetResource() : nullptr;
            FRHITexture* texture = rt_resource ? rt_resource->GetShaderResourceTexture() : nullptr;
            if (texture != nullptr) {
                FIntPoint size;
                auto flags = setupRenderResource(rt_resource, params_[i].get(), results_[i].get(), size);
                const FIntRect rect(0, 0, size.X, size.Y);

                // FRenderTarget::ReadPixels flushes game-thread commands; use RHI readback on the render thread.
                if (!params_[i]->pixels_as_float)
                    RHICmdList.ReadSurfaceData(texture, rect, results_[i]->bmp, flags);
                else if (texture->GetDesc().Format == PF_FloatRGBA) {
                    RHICmdList.Transition(FRHITransitionInfo(texture, ERHIAccess::SRVMask, ERHIAccess::CopySrc));
                    RHICmdList.ReadSurfaceFloatData(texture, rect, results_[i]->bmp_float, flags);
                    RHICmdList.Transition(FRHITransitionInfo(texture, ERHIAccess::CopySrc, ERHIAccess::SRVMask));
                }
            }

            results_[i]->time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
        }

        req_size_ = 0;
        params_ = nullptr;
        results_ = nullptr;

        wait_signal_->signal();
    }
}
