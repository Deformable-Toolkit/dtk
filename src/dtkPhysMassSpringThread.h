#ifndef DTK_PHYSMASSSPRINGTHREAD_H
#define DTK_PHYSMASSSPRINGTHREAD_H

#include <memory>
#include <boost/utility.hpp>
#include "dtkPhysTetraMassSpring.h"

namespace dtk
{
	class dtkPhysMassSpringThread : public dtkPhysTetraMassSpring
	{
		//螺纹弹簧 
	public:
		enum Orientation
		{
			UP = 0,
			DOWN,
			LEFT,
			RIGHT,
			FRONT,
			BACK
		};

		typedef std::shared_ptr<dtkPhysMassSpringThread> Ptr;

		static Ptr New(double interval, int length, dtkT3<double> firstPos, Orientation ori, double mass,
					   double edgeStiff, double bendStiff, double torsionStiff,
					   double edgeDamp, double extraEdgeDamp, double bendDamp, double torsionDamp)
		{
			return Ptr(new dtkPhysMassSpringThread(interval, length, firstPos, ori, mass, edgeStiff,
												   bendStiff, torsionStiff, edgeDamp, extraEdgeDamp, bendDamp, torsionDamp));
		};

	public:
		~dtkPhysMassSpringThread();

		bool ApplyImpulse(double timeslice); //应用冲量

		void constructThreadMesh(); //构建螺纹弹簧网格

		void constructTetraMesh(); //构建四面体网格

		void addEdgeSpring(); //边弹簧

		void addExtraEdgeSpring();

		void addBendSpring(); //弯曲弹簧

		void addTorsionSpring(); //扭曲弹簧

		dtk::dtkStaticTetraMesh::Ptr getTetraMesh()
		{
			return mTetraMeshPtr;
		}

		size_t GetNumberOfSegments()
		{
			return mLength;
		}

		double GetInterval()
		{
			return mInterval;
		}

		//计算冲量，并控制传播.
		
		void ControlEndPropagate(double timeslice, int range = 30);

		//冲量传播，给不同的质点.

		void ImpulsePropagate(dtkT3<double> impulse, dtkID pointID, dtkID ori, size_t range = 30);

	private:
		dtkPhysMassSpringThread(double interval, int length, dtk::dtkT3<double> firstPos, Orientation ori, double mass,
								double edgeStiff, double bendStiff, double torsionStiff,
								double edgeDamp, double extraEdgeDamp, double bendDamp, double torsionDamp);

		dtk::dtkStaticTetraMesh::Ptr mTetraMeshPtr;

		double mEdgeStiff; //边刚性系数，弹性系数 

		double mExtraEdgeStiff; // 额外边弹簧刚性系数，弹性系数 

		double mBendStiff; // 弯弹簧刚性系数，弹性系数 

		double mTorsionStiff; //扭转刚性 

		double mEdgeDamp; //边阻尼 

		double mExtraEdgeDamp; //额外边阻尼 

		double mBendDamp; //弯曲阻尼

		double mTorsionDamp; //扭转阻尼 

		size_t mLength; //长度 

		dtkT3<double> mFirstPos;

		double mInterval; //间隔 

		double mRotateInterval; //旋转间隔 

		dtk::dtkID mOrientation; //朝向 
	};
};

#endif
