#ifndef GUIDEWIRE_H
#define  GUIDEWIRE_H

// dtk中的头文件
#include "dtkPhysMassPoint.h"

// std中的头文件
#include <vector>
using namespace std;

namespace dtk
{
	class guideWire:public boost::noncopyable
	{
	//  公有成员函数
	public :
		typedef std::shared_ptr<guideWire>Ptr;
		static Ptr New()
		{
			return Ptr(new guideWire());
		}

		// 导丝的动态更新
		void Update(double timeslice,  ItrMethod method = Euler, dtkID iteration = 0);
		void UpdateMassPoints(double timeslice, ItrMethod  method = Euler, dtkID iteration = 0);
		void PreUpdate(double timeslice);

		// 导丝的参数设置
		void SetPoints(dtkPoints::Ptr points);
		void SetSegInterval(double segInterval);
		void SetTipSegInterval(double tipSegInterval);
		void SetLastTipID(dtkID lastTipID);
		void SetBendModulus(double bendModulus);
		void SetTipBendModulus(double tipBendModulus);
		void Set3DBendModulus(double threeDBendModulus);
		void SetTipOriginAngle(const vector<double>& tipOriginAngle);
		void SetMass(double mass);
		void SetPointResistence(double pointResistence);
		void SetTipPointResistence(double tipPointResistence);
		void SetContactForces(dtkID id, const dtkDouble3 & force);
		void SetCollisionFlag(dtkID id, bool flag);
		void ResetContactForces();
		void ResetCollisionFlag();
		void SetGuideWireStartPoint(const GK::Point3 & point);
		void SetGuideWireStartDirection(const GK::Vector3 & direction);
		

		// 获取导丝的参数
		dtkPoints::Ptr GetPoints() const;
		double GetSegInterval() const;
		double GetTipSegInterval() const;
		dtkID GetLastTipID() const;
		double GetBendModulus() const;
		double Get3DBendModulus() const;
		vector<double> GetTipOriginAngle() const;
		double GetMass() const;
		double GetPointResistence() const;
		double GetTipPointResistence() const;
		double GetTipBendModulus() const;
		
		// 导丝的功能函数
		void RemovePoint();		// 移除导丝体部最后的一个质点的坐标
		void AddPoint(const GK::Point3 & point);				// 往导丝体部的后面增加一个质点的坐标
		void DynamicGuideWirePoint();								// 根据导丝的运动来增删质点
		
		// 导丝的模型函数
		void ConstructGuideWireMassPoints() ;				// 将导丝建立成离散的质点模型
		dtkPhysMassPoint* GetMassPoint(dtkID id)  const;					
		dtkID AddMassPoint(const dtkT3<double>& vel);
		bool RemoveMassPoint();										// 移除一个导丝质点
		void ResistStretch(double timeslice);						// 导丝的抗拉伸
		void AddBendForce(double timeslice);					// 导丝受到的弯曲力
		void ResistOverBend(double timeslice);				// 用位置法来限制导丝尖端的弯曲
		void Add3DBendForce(double timeslice);				// 导丝受到的平面弯曲力
		void AddTwistForce(double twistAngle);				// 导丝受到的扭曲力
		void AddContactForce();								// 导丝在运动的过程受到血管壁的接触力
		
		// 对导丝的受力、位置和速度进行操作
		void AddForce(const dtkT3<double> & force, dtkID directionID, bool isPush);   // 用户推拉的力的传播函数
		// 用户对导丝的外力操作
		void ApplyExternalForce(const dtkT3<double> & force);			//	用户的推、拉外力
		void RotateMassPoint(dtkPoints::Ptr  points, dtkID rotatePointID, dtkID axisPointID1, dtkID axisPointID2, double angle);	

		// 静态数据类型转换函数
		static dtkT3<double> vector3TodtkT3(const GK::Vector3& v3)
		{
			return dtkT3<double>(v3.x(), v3.y(), v3.z());
		}

	public:
		vector<dtkDouble3> mContactForces;						// 血管壁对导丝的接触力


	// 私有成员函数
	private:
		guideWire();

	// 私有数据成员
		double mSegInterval;				// 导丝的原始段长
		double mTipSegInterval;			// 导丝尖端的段长
		dtkPoints::Ptr mPts;					// 导丝的质点坐标
		dtkID mLastTipID;						// 导丝尖端最末段的ID
		double mBendModulus;			// 导丝体部分的弯曲模数
		double mTipBendModulus;		// 导丝尖端部分的二维弯曲模数
		double m3DBendModulus;		// 导丝尖端部分的3D弯曲模数
		vector<double> mTipOriginAngle;		// 导丝尖端部分的原始角度
		double mMass;										// 导丝的质量
		double mTimeslice;                              //  迭代的时间片大小
		double mDefaultPointResistence;      // 导丝体部在血管中的阻尼系数
		double mDefaultTipPointResistence; // 导丝尖端在血管中的阻尼系数
		double mDefaultPointDamp;	            // 导丝在血管中速度的降低倍数 
		double mTwistSlice;								// 导丝在血管中的扭曲角度片大小
		double mDefaultGravityAccel;				// 默认的重力加速度
		double mIncompleteSegLength;			// 不完整的导丝段长
		GK::Point3 mGuideWireStartPoint;           // 导丝的起始位置
		GK::Vector3 mGuideWireStartDirection;	// 导丝的起始方向
		vector<dtkPhysMassPoint *>  mMassPoints;			// 导丝的质点
		vector<bool> mMassPointsCollisionFlag;					// 质点与血管壁发生碰撞的Flag，其中发生碰撞为true，否则为false
		

	};
}
#endif