#ifndef _libUWBX_H
#define _libUWBX_H

#ifdef WIN32
#define UWBX_API	__stdcall
#else
#define UWBX_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

int UWBX_API UWBX_startTOF();
int UWBX_API UWBX_startTDOA();
int UWBX_API UWBX_setAnchorPos( const char* strAnchorPos );
int UWBX_API UWBX_setMasterAnchorPos( const char* strAnchorPos );
int UWBX_API UWBX_setParam( const char* params );
int UWBX_API UWBX_dataUpdate( const char* strData );
int UWBX_API UWBX_getPosition( int index, int* px, int* py /*cm*/, int* TagId);
int UWBX_API UWBX_locate( const char* strData, int* px, int* py /*cm*/, int useKF );
int UWBX_API UWBX_destroy(int tagId);
int UWBX_API UWBX_destroyAll();

#ifdef __cplusplus
}
#endif

#endif //_libUWBX_H
