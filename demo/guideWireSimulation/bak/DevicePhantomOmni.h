#ifndef DEVICEPHANTOMOMNI_H
#define DEVICEPHANTOMOMNI_H

#include <cmath>
#include <memory>


#include <HD/hd.h>
#include <HDU/hdu.h>
#include <HDU/hduVector.h>


typedef struct
{
	hduVector3Dd position;
	hduVector3Dd gimbalAngle;
	HDdouble transform[16];
	bool button1;
	bool button2;	
} HapticDisplayState;



class DevicePhantomOmni
{
public:
	typedef std::shared_ptr<DevicePhantomOmni>Ptr;
	static Ptr New()
	{
		return Ptr(new DevicePhantomOmni);
	}
	DevicePhantomOmni();
	void InitHD();
	void SetHapticForce(hduVector3Dd force, HDulong forceRate);
	void SetHapticRate(HDulong hapticRate);
	
	HapticDisplayState * GetCurrentDisplayState();
	HapticDisplayState * GetLastDisplayState();

	HDdouble * GetWorkspaceModel(const HDdouble * modelMatrix, const HDdouble *projMatrix);
	~DevicePhantomOmni();
	static HDCallbackCode HDCALLBACK TouchScene(void *pUserData);
	static HDCallbackCode HDCALLBACK CopyHapticDisplayState(void *pUserData);
	static HHD ghHD;
	static hduVector3Dd mForce;
	static hduVector3Dd mLastForce;
	static hduVector3Dd mCurrentHaticForce;
	static HDulong mForceRate;
	static HDulong mHapticRate;
	static bool mPeriodStartFlag;
	static int	mPeriodHapticTimes;



private:
	HapticDisplayState mCurrentDisplayState;
	HapticDisplayState mLastDisplayState;
	hduVector3Dd mAngularVelocity;

	HDdouble mWorkspaceModel[16];

	HDSchedulerHandle hUpdateDeviceCallback;
	
	
};

#endif
