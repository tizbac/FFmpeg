#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"
#include "libavutil/common.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/pixelutils.h"
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define OFFSET(x) offsetof(FrameInterpolationContext, x)


typedef struct {
    AVFrame * prev;
    int blocksize;
    int target_fps;
    av_pixelutils_sad_fn sad;
} FrameInterpolationContext;

static const AVOption frameinterp_options[] = {
    { "fps" , "Frames per second of the output video", OFFSET(target_fps), AV_OPT_TYPE_INT, {.i64=60}, 30, 120, .flags = FLAGS },
    { "blocksize" , "Block size bits to use ( 3=8, 4=16, 5=32, 6=64, 7=128 ) ", OFFSET(blocksize), AV_OPT_TYPE_INT, {.i64=4}, 3, 7, .flags = FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(frameinterp);

static av_cold int init(AVFilterContext *ctx)
{
    FrameInterpolationContext* interp = ctx->priv;
    interp->sad = av_pixelutils_get_sad_fn(interp->blocksize,interp->blocksize,1,interp);
    interp->prev = NULL;
    
    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    FrameInterpolationContext* interp = ctx->priv;
    av_frame_free(&interp->prev);
}

static av_always_inline int sad_wxh(const uint8_t *src1, ptrdiff_t stride1,
                                    const uint8_t *src2, ptrdiff_t stride2,
                                    int w, int h)
{
    int x, y, sum = 0;

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++)
            sum += abs(src1[x] - src2[x]);
        src1 += stride1;
        src2 += stride2;
    }
    return sum;
}

static int filter_frame(AVFilterLink *link, AVFrame *in)
{
    FrameInterpolationContext* interp = link->dst->priv;
    AVFilterLink *outlink = link->dst->outputs[0];
    AVFrame * frame1 = interp->prev;
    AVFrame * frame2 = in;
    
    int blocksize = 1<<interp->blocksize;
    int bX,bY,i,X,Y;
    int diamondresults[9];
    int* vectorsx = NULL; 
    int* vectorsy = NULL;
    int* errorvector = NULL;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(frame2->format);

    
    int stride = in->linesize[0];
    if ( frame1 )
    {
        vectorsx = av_malloc(sizeof(int)*(frame1->width/blocksize)*(frame1->height/blocksize));
        vectorsy = av_malloc(sizeof(int)*(frame1->width/blocksize)*(frame1->height/blocksize));
        errorvector = av_malloc(sizeof(int)*(frame1->width/blocksize)*(frame1->height/blocksize));
        for ( bX = 0; (bX+1)*blocksize < frame1->width; bX++ )
        {
            for ( bY = 0; (bY+1)*blocksize < frame1->height; bY++ )
            {
                int cX = bX*blocksize;
                int cY = bY*blocksize;
                int cX2 = cX;
                int cY2 = cY;
                //Diamond search pixels offsets
                /*
                * 0 0
                * -2 0
                * 2 0
                * 0 2
                * 0 -2
                * 1 1
                * -1 1
                * -1 -1
                * 1 -1
                * 
                */
#define DIAMOND(index, offsetx, offsety) diamondresults[index] = sad_wxh(frame1->data[0]+cY*stride+cX,stride,frame2->data[0]+(cY2+offsety)*stride+cX2+offsetx,stride,blocksize,blocksize)
                
                while ( 1 )
                {
                    int mindiff = 0;
                    int mindiffv = 9999999.0;
                    DIAMOND(0,0,0);
                    DIAMOND(1,-2,0);
                    DIAMOND(2,2,0);
                    DIAMOND(3,0,2);
                    DIAMOND(4,0,-2);
                    DIAMOND(5,1,1);
                    DIAMOND(6,-1,1);
                    DIAMOND(7,-1,-1);
                    DIAMOND(8,1,-1);
                    for ( int k = 0; k < 9; k++ )
                    {
                        if ( diamondresults[k] < mindiffv )
                        {
                            mindiffv = diamondresults[k];
                            mindiff = k;
                        }
                    }
                    if ( mindiff != 0 ) //Not center
                    {
                        switch(mindiff) {
                            case 1:
                                cX2 -=2;
                                break;
                            case 2:
                                cX2 +=2;
                                break;
                            case 3:
                                cY2 +=2;
                                break;
                            case 4:
                                cY2 -=2;
                                break;
                            case 5:
                                cX2 +=1;
                                cY2 +=1;
                                break;
                            case 6:
                                cX2 -=1;
                                cY2 +=1;
                                break;
                            case 7:
                                cX2 -=1;
                                cY2 -=1;
                                break;
                            case 8:
                                cX2 +=1;
                                cY2 -=1;
                                break;
                        }
                        continue;
                    }
                    //mindiff == 0
                    
                    DIAMOND(0,0,0);
                    DIAMOND(1,-1,0);
                    DIAMOND(2,1,0);
                    DIAMOND(3,0,1);
                    DIAMOND(4,0,-1);
                    
                    mindiff = 0;
                    mindiffv = 999999999.0;
                    
                    for ( int k = 0; k < 5; k++ )
                    {
                        if ( diamondresults[k] < mindiffv )
                        {
                            mindiffv = diamondresults[k];
                            mindiff = k;
                        }
                    }
                    
                    switch(mindiff) {
                        case 0:
                            break;
                        case 1:
                            cX2 -=1;
                            break;
                        case 2:
                            cX2 +=1;
                            break;
                        case 3:
                            cY2 +=1;
                            break;
                        case 4:
                            cY2 -=1;
                            break;
                    }
                    if ( cX2 < 0 )
                        cX2 = 0;
                    if ( cX2 >= frame1->width )
                        cX2 = frame1->width -1;
                    if ( cY2 < 0 )
                        cY2 = 0;
                    if ( cY2 >= frame1->height )
                        cY2 = frame1->height -1;
                    //printf("Vec: %d %d\n",cX2-cX,cY2-cY);
                    vectorsx[bY*(frame1->width/blocksize)+bX] = cX2-cX;
                    vectorsy[bY*(frame1->width/blocksize)+bX] = cY2-cY;
                    errorvector[bY*(frame1->width/blocksize)+bX] = mindiffv;
                    //debug2->drawLine(centerX, centerY, centerX2, centerY2, mindiffv > 5.0 ? 255 : 0,mindiffv > 5.0 ? 0 : 255,0);
                    
                    
                    break;
                }
                
            }
            
        }
        
        
        printf("PTS1: %lld PTS2: %lld\n",frame1->pts,frame2->pts);

        
        float factor = 0.5;
        AVFrame* out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        av_frame_copy_props(out, in);
        
        if ( desc )
        {
            for ( i = 0; i < 3; i++ )
            {
                if ( i > 0 )
                    memcpy(out->data[i],frame1->data[i],out->linesize[i]*FF_CEIL_RSHIFT(out->height, desc->log2_chroma_h));
                else
                    memcpy(out->data[i],frame1->data[i],out->linesize[i]*out->height);
            }
        }
        ff_filter_frame(outlink, out);
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        av_frame_copy_props(out, in);
        out->pts = (frame2->pts+frame1->pts)/2;
        if ( desc )
        {
            for ( i = 0; i < 3; i++ )
            {
                if ( i > 0 )
                    memcpy(out->data[i],frame1->data[i],out->linesize[i]*FF_CEIL_RSHIFT(out->height, desc->log2_chroma_h));
                else
                    memcpy(out->data[i],frame1->data[i],out->linesize[i]*out->height);
            }
        }
        
#define VX(BX,BY) vectorsx[BY*(frame1->width/blocksize)+BX]
#define VY(BX,BY) vectorsy[BY*(frame1->width/blocksize)+BX]
#define VE(BX,BY) errorvector[BY*(frame1->width/blocksize)+BX]
#define TTH 0.2
        for ( bX = 1; (bX+2)*blocksize < frame1->width; bX++ )
        {
            for ( bY = 1; (bY+2)*blocksize < frame1->height; bY++ )
            {
                
                float t1 = atan2(VY(bX,bY),VX(bX,bY));
                float t[] = { atan2(VY(bX+1,bY),  VX(bX+1,bY)), 
                              atan2(VY(bX-1,bY),  VX(bX-1,bY)), 
                              atan2(VY(bX+1,bY+1),VX(bX-1,bY+1)), 
                              atan2(VY(bX,  bY+1),VX(bX,  bY+1)), 
                              atan2(VY(bX-1,bY+1),VX(bX-1,bY+1)), 
                              atan2(VY(bX+1,bY-1),VX(bX+1,bY-1)), 
                              atan2(VY(bX,  bY-1),VX(bX  ,bY-1)),
                              atan2(VY(bX-1,bY-1),VX(bX-1,bY-1))
                };
                
                
                int allsimiliar = 1;
                int k,j;
                float avg = 0;
                
                if ( VE(bX,bY) > 255*blocksize*blocksize/32 )
                {
                    VY(bX,bY) = (VY(bX+1,bY)+VY(bX-1,bY)+VY(bX+1,bY+1)+VY(bX,  bY+1)+VY(bX-1,bY+1)+VY(bX+1,bY-1)+VY(bX,  bY-1)+VY(bX-1,bY-1))/8;
                    VX(bX,bY) = (VX(bX+1,bY)+VX(bX-1,bY)+VX(bX+1,bY+1)+VX(bX,  bY+1)+VX(bX-1,bY+1)+VX(bX+1,bY-1)+VX(bX,  bY-1)+VX(bX-1,bY-1))/8;
                    continue;
                }
                for ( k = 0; k < 8; k++ )
                {
                    for ( j = 0; j < 8; j++ )
                    {
                        if ( fabs(t[j]-t[k]) > TTH )
                        {
                            allsimiliar = 0;
                            break;
                        }
                        
                    }
                    avg += t[k];
                }
                
                avg /= 8.0f;
                
                if ( allsimiliar )
                {
                    if ( fabs(avg-t1) > TTH )
                    {
                        VY(bX,bY) = (VY(bX+1,bY)+VY(bX-1,bY)+VY(bX+1,bY+1)+VY(bX,  bY+1)+VY(bX-1,bY+1)+VY(bX+1,bY-1)+VY(bX,  bY-1)+VY(bX-1,bY-1))/8;
                        VX(bX,bY) = (VX(bX+1,bY)+VX(bX-1,bY)+VX(bX+1,bY+1)+VX(bX,  bY+1)+VX(bX-1,bY+1)+VX(bX+1,bY-1)+VX(bX,  bY-1)+VX(bX-1,bY-1))/8;
                        
                    }
                    
                }
            }
            
        }
        
        for ( bX = 0; (bX+1)*blocksize < frame1->width; bX++ )
        {
            for ( bY = 0; (bY+1)*blocksize < frame1->height; bY++ )
            {
                int vxPrev,vyPrev;
                int vxNext,vyNext;
                int vX = vectorsx[bY*(frame1->width/blocksize)+bX]*factor;
                int vY = vectorsy[bY*(frame1->width/blocksize)+bX]*factor;
                if ( bX > 0 )
                    vxPrev = VX(bX-1,bY);
                else
                    vxPrev = vX;
                if ( bY > 0 )
                    vyPrev = VY(bX,bY-1);
                else
                    vyPrev = vY;
                
                
                for ( X = bX*blocksize; X < (bX+1)*blocksize; X++ )
                {
                    for ( Y = bY*blocksize; Y < (bY+1)*blocksize; Y++ )
                    {
                        int vXI,vYI;
                        if ( X < blocksize/2 )
                            vXI = vxPrev+(vX-vxPrev)*((float)(X)/(float)(blocksize/2));
                        else
                            vXI = vX+(vxNext-vX)*((float)(X-blocksize/2)/(float)(blocksize/2));
                        if ( Y < blocksize/2 )
                            vYI = vyPrev+(vY-vyPrev)*((float)(Y)/(float)(blocksize/2));
                        else
                            vYI = vY+(vyNext-vY)*((float)(Y-blocksize/2)/(float)(blocksize/2));
                        uint8_t p = frame1->data[0][stride*Y+X];
                        if ( Y+vYI < out->height && Y+vYI >= 0 && X+vXI < out->width && X+vXI >= 0 )
                            out->data[0][stride*(Y+vYI)+X+vXI] = p;
                    }
                }
            }
        }
        
        
        ff_filter_frame(outlink, out);
        av_free(vectorsx);
        av_free(vectorsy);
        
    }
    
    
    
    interp->prev = frame2;
    return 0;
}

static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pix_fmts[] = {
        AV_PIX_FMT_YUV420P,  AV_PIX_FMT_YUV422P,  AV_PIX_FMT_YUV444P,  AV_PIX_FMT_YUV410P,
        AV_PIX_FMT_YUV411P,  AV_PIX_FMT_YUV440P,  AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ422P,
        AV_PIX_FMT_YUVJ444P, AV_PIX_FMT_YUVJ440P, AV_PIX_FMT_NONE
    };
    AVFilterFormats *fmts_list = ff_make_format_list(pix_fmts);
    if (!fmts_list)
        return AVERROR(ENOMEM);
    return ff_set_common_formats(ctx, fmts_list);
}

static const AVFilterPad frameinterp_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
    { NULL }
};

static const AVFilterPad frameinterp_outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
    },
    { NULL }
};

AVFilter ff_vf_frameinterp = {
    .name          = "frameinterp",
    .description   = NULL_IF_CONFIG_SMALL("Generate sub-frames to get higher framerate"),
    .priv_size     = sizeof(FrameInterpolationContext),
    .init          = init,
    .uninit        = uninit,
    .query_formats = query_formats,
    .inputs        = frameinterp_inputs,
    .outputs       = frameinterp_outputs,
    .priv_class    = &frameinterp_class,
};