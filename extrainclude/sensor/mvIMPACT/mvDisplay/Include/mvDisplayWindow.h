//---------------------------------------------------------------------------
#ifndef mvDisplayWindowH
#define mvDisplayWindowH mvDisplayWindowH
//---------------------------------------------------------------------------
#include <mvDisplay/Include/mvDisplay.h>

#if defined(__cplusplus)
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

/// \brief Display window handle for C compliant interface.
typedef void* HDISP;

/// \brief Prototype for the callback function that can be passed to \b mvDispWindowSetMessageHandler()
typedef void ( *TPreProcessMessage )( HDISP hDisp, UINT message, WPARAM wParam, LPARAM lParam );

void               MV_DISPLAY_API_CALL mvDispWindowDestroy( HDISP hDisp );
WindowHandle       MV_DISPLAY_API_CALL mvDispWindowGetWindowHandle( HDISP hDisp );
TDisp*             MV_DISPLAY_API_CALL mvDispWindowGetDisplayHandle( HDISP hDisp );
HDISP              MV_DISPLAY_API_CALL mvDispWindowCreate( const char* title );
int                MV_DISPLAY_API_CALL mvDispWindowSetRefreshTime( HDISP hDisp, int time_ms );
void               MV_DISPLAY_API_CALL mvDispWindowShow( HDISP hDisp );
void               MV_DISPLAY_API_CALL mvDispWindowSetMessageHandler( HDISP hDisp, TPreProcessMessage handler );
TPreProcessMessage MV_DISPLAY_API_CALL mvDispWindowGetMessageHandler( HDISP hDisp );

#if defined(WRAP_DOTNET) || defined(MVIMPACT_ACQUIRE_DISPLAY_H_) || defined(DOXYGEN_CPP_DOCUMENTATION)
} // namespace display
} // namespace acquire
} // namespace mvIMPACT
#endif // #if defined(WRAP_DOTNET) || defined(MVIMPACT_ACQUIRE_DISPLAY_H_) || defined(DOXYGEN_CPP_DOCUMENTATION)

#if defined(__cplusplus)
}
#endif

#endif // mvDisplayWindowH