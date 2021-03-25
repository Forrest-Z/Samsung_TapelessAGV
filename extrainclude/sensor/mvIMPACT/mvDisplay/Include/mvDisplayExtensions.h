//---------------------------------------------------------------------------
#ifndef mvDisplayExtensionsH
#ifndef DOXYGEN_SHOULD_SKIP_THIS
#   define mvDisplayExtensionsH mvDisplayExtensionsH
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------
#include <mvDisplay/Include/mvDisplay.h>
#ifdef __cplusplus
#   include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#else
#   include <mvDeviceManager/Include/mvDeviceManager.h>
#endif // __cplusplus

#if defined(WRAP_DOTNET) || defined(MVIMPACT_ACQUIRE_DISPLAY_H_) || defined(DOXYGEN_CPP_DOCUMENTATION)
namespace mvIMPACT
{
namespace acquire
{
namespace display
{
#endif // #if defined(WRAP_DOTNET) || defined(MVIMPACT_ACQUIRE_DISPLAY_H_) || defined(DOXYGEN_CPP_DOCUMENTATION)

#ifdef __cplusplus
extern "C" {
#endif

void MV_DISPLAY_API_CALL mvDispConvertFormat( TImageBufferPixelFormat pixelFormat, TFormatFlags* pFormat, int* pBitsPerPixel );
void MV_DISPLAY_API_CALL mvDispSetImageFromImageBuffer( TDisp* pDisp, const ImageBuffer* pBuf );

#ifdef __cplusplus
}
#endif // __cplusplus

#ifdef __cplusplus
//-----------------------------------------------------------------------------
/// \brief Sets the next image to display.
/**
 *  This function can deal with any pixel format supported by mvIMPACT Acquire.
 */
inline void mvDispSetImageFromRequest(
    /// [in] A handle to a display structure obtained
    /// from a successful call to \b mvDispInit().
    TDisp* pDisp,
    /// [in] The request object that shall be displayed.
    const mvIMPACT::acquire::Request* pRequest )
//-----------------------------------------------------------------------------
{
if( pRequest )
{
    const size_t MAX_PLANE_CNT = 3;
    const void*  dataPtrArray[MAX_PLANE_CNT];
    char*        p = ( char* )pRequest->imageData.read();
    int          bitsPerPixel = 8;
    TFormatFlags format = ffMono;

    mvDispConvertFormat( pRequest->imagePixelFormat.read(), &format, &bitsPerPixel );
    switch( pRequest->imagePixelFormat.read() )
    {
    case ibpfRGBx888Planar:
        dataPtrArray[2] = p + pRequest->imageChannelOffset.read( 0 );
        dataPtrArray[1] = p + pRequest->imageChannelOffset.read( 1 );
        dataPtrArray[0] = p + pRequest->imageChannelOffset.read( 2 );
        break;
    case ibpfYUV422Planar:
        dataPtrArray[0] = p + pRequest->imageChannelOffset.read( 0 );
        dataPtrArray[1] = p + pRequest->imageChannelOffset.read( 1 );
        dataPtrArray[2] = p + pRequest->imageChannelOffset.read( 2 );
        break;
    default:
        dataPtrArray[0] = p;
        break;
    }
    mvDispSetImageEx( pDisp, dataPtrArray, MAX_PLANE_CNT, format, pRequest->imageWidth.read(), pRequest->imageHeight.read(), bitsPerPixel, pRequest->imageLinePitch.read() );
}
else
{
    mvDispSetImage( pDisp, 0, 0, 0, 8, 0 );
}
}
#endif // #ifdef __cplusplus

#if defined(WRAP_DOTNET) || defined(MVIMPACT_ACQUIRE_DISPLAY_H_) || defined(DOXYGEN_CPP_DOCUMENTATION)
} // namespace display
} // namespace acquire
} // namespace mvIMPACT
#endif // #if defined(WRAP_DOTNET) || defined(MVIMPACT_ACQUIRE_DISPLAY_H_) || defined(DOXYGEN_CPP_DOCUMENTATION)

#endif // mvDisplayExtensionsH
