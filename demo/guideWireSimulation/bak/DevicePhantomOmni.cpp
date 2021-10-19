#include "DevicePhantomOmni.h"

HHD DevicePhantomOmni::ghHD;
hduVector3Dd DevicePhantomOmni::mForce;
hduVector3Dd DevicePhantomOmni::mLastForce;
hduVector3Dd DevicePhantomOmni::mCurrentHaticForce;
HDulong DevicePhantomOmni::mForceRate;
HDulong DevicePhantomOmni::mHapticRate;
bool DevicePhantomOmni::mPeriodStartFlag;
int	 DevicePhantomOmni::mPeriodHapticTimes;

DevicePhantomOmni::DevicePhantomOmni()
{
	ghHD = HD_INVALID_HANDLE;
	hUpdateDeviceCallback = HD_INVALID_HANDLE;
	mHapticRate = 1000;
	mForceRate = 1000;
	mCurrentHaticForce = hduVector3Dd(0, 0, 0);
	mPeriodStartFlag = false;
	mPeriodHapticTimes = 0;
}

void DevicePhantomOmni::InitHD()
{
	ghHD = hdInitDevice(HD_DEFAULT_DEVICE);

	hdEnable(HD_FORCE_OUTPUT);
	hUpdateDeviceCallback = hdScheduleAsynchronous(
		TouchScene, 0, HD_MAX_SCHEDULER_PRIORITY);

	hdStartScheduler();
}

void DevicePhantomOmni::SetHapticForce(hduVector3Dd force, HDulong forceRate)
{
	// 判断force是不是超过了phantom Omni的输出最大值，如果超过则对超过部分进行截短
	HDdouble maxForce;
	hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE , &maxForce);
	if (hduVecMagnitude(force) > maxForce)
	{
		hduVecNormalizeInPlace(force);
		hduVecScaleInPlace(force, maxForce);
	}
	mLastForce = mForce;
	mForce = force;
	mForceRate = forceRate;
	mPeriodStartFlag = true;
}

void DevicePhantomOmni::SetHapticRate(HDulong hapticRate)
{
	mHapticRate = hapticRate;
	hdSetSchedulerRate(hapticRate);
}

HDCallbackCode HDCALLBACK DevicePhantomOmni::TouchScene(void *pUserData)
{
	//根据SetHapticForce来产生连续的力触觉
	//hduVector3Dd torqueV;

	if (mPeriodStartFlag)
		mPeriodHapticTimes = 0;
	mPeriodStartFlag = false;
	mPeriodHapticTimes ++;
	hdBeginFrame(ghHD);
	HDulong times = mHapticRate / mForceRate;
	hduVector3Dd clampForce = (mForce - mLastForce) / times;
	if (mPeriodHapticTimes <= times)
		mCurrentHaticForce = mCurrentHaticForce + clampForce;
	hdSetDoublev(HD_CURRENT_FORCE, mCurrentHaticForce);
	hdSetDoublev(HD_CURRENT_TORQUE, hduVector3Dd(5, 5, 5)); 
	//hdGetDoublev(HD_CURRENT_TORQUE, torqueV);
	hdEndFrame(ghHD);
	return HD_CALLBACK_CONTINUE;
}

HDCallbackCode HDCALLBACK DevicePhantomOmni::CopyHapticDisplayState(void *pUserData)
{
	int currentButtons;
	HapticDisplayState *pState = (HapticDisplayState *) pUserData;

	hdGetDoublev(HD_CURRENT_POSITION, pState->position);
	hdGetDoublev(HD_CURRENT_TRANSFORM, pState->transform);
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, pState->gimbalAngle);
	//hdGetDoublev(HD_CURRENT_TORQUE, torqueV);
	 
	hdGetIntegerv(HD_CURRENT_BUTTONS, &currentButtons);
	pState->button1 = currentButtons & HD_DEVICE_BUTTON_1;
	pState->button2 = currentButtons & HD_DEVICE_BUTTON_2;

	return HD_CALLBACK_DONE;
}


HapticDisplayState * DevicePhantomOmni::GetCurrentDisplayState()
{
	mLastDisplayState = mCurrentDisplayState;
	hdScheduleSynchronous(CopyHapticDisplayState, &mCurrentDisplayState, HD_DEFAULT_SCHEDULER_PRIORITY);
	return &mCurrentDisplayState;
}

HapticDisplayState * DevicePhantomOmni::GetLastDisplayState()
{
	return &mLastDisplayState;
}

HDdouble * DevicePhantomOmni::GetWorkspaceModel(const HDdouble * modelMatrix, const HDdouble *projMatrix)
{
	hduMapWorkspaceModel(modelMatrix, projMatrix, mWorkspaceModel);
	return mWorkspaceModel;
}


DevicePhantomOmni::~DevicePhantomOmni()
{
	// nothing
}

