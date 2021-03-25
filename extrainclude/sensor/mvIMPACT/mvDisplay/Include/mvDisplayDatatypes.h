//-----------------------------------------------------------------------------
#ifndef mvDisplayDatatypesH
#if !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)
#define mvDisplayDatatypesH mvDisplayDatatypesH
#endif // DOXYGEN_SHOULD_SKIP_THIS && WRAP_ANY
//-----------------------------------------------------------------------------

#if !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)
#   ifdef _WIN32
#       ifdef __BORLANDC__
#           pragma option push -b // force enums to the size of integer
#       endif // __BORLANDC__
#   endif // _WIN32
#endif // #if !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)

#ifdef __cplusplus
extern "C" {
#endif

#if defined(WRAP_DOTNET) || defined(MVIMPACT_ACQUIRE_DISPLAY_H_) || defined(DOXYGEN_CPP_DOCUMENTATION)
namespace mvIMPACT
{
namespace acquire
{
namespace display
{
#endif // #if defined(WRAP_DOTNET) || defined(MVIMPACT_ACQUIRE_DISPLAY_H_) || defined(DOXYGEN_CPP_DOCUMENTATION)

//---------------------------------------------------------------------------
/// \brief Defines valid error values for this module.
/// \ingroup Display
enum TImageDisplayError
//---------------------------------------------------------------------------
{
    /// \brief No error occurred since the last time the error code was queried.
    IDE_NoError = 0,
    /// \brief There is not enough memory available to perform the requested operation.
    IDE_OutOfMemory,
    /// \brief The update failed. This is an internal error.
    IDE_UpdateFailed,
    /// \brief One or more of the handles passed to a function where invalid.
    IDE_InvalidHandle,
    /// \brief Unspecified internal error.
    IDE_InternalError,
    /// \brief Calling a DirectDraw&reg; function did fail.
    IDE_DirectDrawAccessFailed
};

//---------------------------------------------------------------------------
/// \brief Defines valid display modes.
/// \ingroup Display
enum TDisplayMode
//---------------------------------------------------------------------------
{
    /// \brief The default display mode.
    /**
     *  This display mode is capable of scaling the image. Scaling however will
     *  result in additional CPU load.
     *
     *  In this mode, the bitmap rectangle will always be scaled to fit into the display rectangle.
     *
     *  In this mode(and in this mode only) overlay callbacks will be executed.
     */
    DM_Default = 0,
    /// \brief The fastest display mode.
    /**
     *  In this mode there will be no scaling and no overlay callbacks but it will
     *  operate with the lowest possible CPU load.
     */
    DM_Fastest,
    /// \brief DirectDraw&reg; mode.
    /**
     *  This mode will use DirectDraw&reg; for displaying the image.
     *
     *  In this mode, the bitmap rectangle will always be scaled to fit into the display rectangle.
     */
    DM_DDrawOverlay,
    /// \brief Fullscreen (Exclusive) DirectDraw&reg; mode.
    /**
     *  This mode will use DirectDraw&reg; exclusive mode for displaying the image.
     *
     *  In this mode, the bitmap rectangle will always be scaled to fit into the display rectangle.
     */
    DM_FullScreen
};

//---------------------------------------------------------------------------
/// \brief Defines valid scaler interpolation modes.
/// \ingroup Display
enum TInterpolationMode
//---------------------------------------------------------------------------
{
    /// \brief Nearest neighbor interpolation (default).
    /**
     *  Fast but with reduced quality.
     */
    IM_NEAREST_NEIGHBOUR = 0,
    /// \brief Linear interpolation.
    /**
     *  Both quality and CPU load will be average.
     */
    IM_LINEAR,
    /// \brief Cubic interpolation.
    /**
     *  Best quality, highest CPU load.
     */
    IM_CUBIC
};

//---------------------------------------------------------------------------
/// \brief Defines valid display pixel formats.
/// \ingroup Display
enum TFormatFlags
//---------------------------------------------------------------------------
{
    /// \brief Valid values for bits per pixel in this format: 15, 16, 24, 32; pData[0] points to the packed image data.
    ffRGB888xPacked,
    /// \brief Valid values for bits per pixel in this format: not used; 8 bit per plane, pData[0] points to the Blue, pData[1] points to the Green, pData[2] points to the Red plane.
    ffRGB888xPlanar,
    /// \brief Valid values for bits per pixel in this format: 8, 10, 12, 14, 16, LSB aligned.
    ffMono,
    /// \brief Valid values for bits per pixel in this format: 16, 20, YUV422: 8-10 bit Y | 8-10 bit U | 8-10 bit Y | 8-10 bit V.
    ffYUY2,
    /// \brief Valid values for bits per pixel in this format: not used, YUV422 planar.
    ffYUV422Planar,
    /// \brief Valid values for bits per pixel in this format: 30, 36, 42 or 48 10-16 bits per color component RGB packed data.
    ffRGB2BytePacked,
    /// \brief Valid values for bits per pixel in this format: 16, 20, YUV422: 8-10 bit U | 8-10 bit Y | 8-10 bit V | 8-10 bit Y.
    ffUYVY,
    /// \brief Valid values for bits per pixel in this format: 12, 8 MSB(1), 4 LSBs(1+2), 8MSB(2).
    ffMonoPacked_V2,
    /// \brief Valid values for bits per pixel in this format: 15, 16, 24, 32; pData[0] points to the packed image data.
    ffBGR888xPacked,
    /// \brief Valid values for bits per pixel in this format: 24 or 48.
    ffUYV444,
    /// \brief Valid values for bits per pixel in this format: 48.
    ffYUV444,
    /// \brief Valid values for bits per pixel in this format: 30 (R=pix&0x3FF, G=(pix>>10)&0x3FF, B=(pix>>20)&3FF).
    ffBGR2BytePacked_V2,
    /// \brief Valid values for bits per pixel in this format: 12, 8 MSB(1), 4 LSB(1) + 4 MSB(2), 8LSB(2).
    ffMonoPacked_V1
};

/// \brief Display structure handle for C compliant interface
typedef struct SDisp TDisp;

#if !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)
typedef enum TImageDisplayError TImageDisplayError;
typedef enum TDisplayMode TDisplayMode;
typedef enum TInterpolationMode TInterpolationMode;
typedef enum TFormatFlags TFormatFlags;
#endif // DOXYGEN_SHOULD_SKIP_THIS && WRAP_ANY

#if defined(WRAP_DOTNET) || defined(MVIMPACT_ACQUIRE_DISPLAY_H_) || defined(DOXYGEN_CPP_DOCUMENTATION)
} // namespace display
} // namespace acquire
} // namespace mvIMPACT
#endif // #if defined(WRAP_DOTNET) || defined(MVIMPACT_ACQUIRE_DISPLAY_H_) || defined(DOXYGEN_CPP_DOCUMENTATION)

#ifdef __cplusplus
}
#endif // __cplusplus

// restore Borland compiler switch 'force enums to the size of integer'
#if !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)
#   ifdef _WIN32
#       ifdef __BORLANDC__
#           pragma option pop
#       endif // __BORLANDC__
#   endif // _WIN32
#endif // !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)

#endif // mvDisplayDatatypesH