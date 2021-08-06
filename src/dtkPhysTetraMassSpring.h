#ifndef DTK_PHYSTETRAMASSSPRING_H
#define DTK_PHYSTETRAMASSSPRING_H

#include "dtkPhysMassSpring.h"
#include "dtkStaticTetraMesh.h"
#include <iostream>

namespace dtk
{
	class dtkPhysTetraMassSpring : public dtkPhysMassSpring
	{
	public:
		typedef std::shared_ptr< dtkPhysTetraMassSpring > Ptr;

		static Ptr New(bool fullUseAltSpring = false, double defaultMass = 2, double defaultK = 0, double defaultDamp = 0, double defaultPointDamp = 1.0, double defaultPointResistence = 2.5, dtkDouble3 defaultGravityAccel = dtkDouble3( 0,0,0 ) )
		{
			return Ptr(new dtkPhysTetraMassSpring(fullUseAltSpring,defaultMass,defaultK,defaultDamp, defaultPointDamp, defaultPointResistence, defaultGravityAccel ));
		}

		void SetTetraMesh(dtkStaticTetraMesh::Ptr newTetraMesh);

		dtkStaticTetraMesh::Ptr GetTetraMesh() { return mTetraMesh; }

        //更新四面体弹簧受力情况， 重写基类函数
        bool PreUpdate(double timeslice, ItrMethod method = Euler, dtkID iteration = 0);

        void DeleteTetra( dtkID i );

		void AddTetra( dtkID i );
		
        //重建四面体
		void ReshapeTetra( dtkID i );

		dtkPoints::Ptr GetOriginalPoints() { return mOriPointsPtr; }

		//dtkT3<T> GetBarycentricWeight(dtkT3<T>, dtkT3<T>, dtkT3<T>,dtkT3<T>);

        /*
        dtkT3<T> GetAltitudePoint1() { return mAltitudePoint1; }
        dtkT3<T> GetAltitudePoint2() { return mAltitudePoint2; }
        dtkT3<T> GetForceA() { return mForceA; }
        dtkT3<T> GetForceB() { return mForceB; }
        dtkT3<T> GetForceC() { return mForceC; }
        dtkT3<T> GetForceD() { return mForceD; }
        */
	public:
		virtual ~dtkPhysTetraMassSpring();

#ifdef DTK_DEBUG
        std::vector< std::pair< dtkT3<double>, dtkT3<double> > > mAltitudeSpringPositions;   
#endif // DTK_DEBUG

	protected:
		dtkPhysTetraMassSpring(bool fullUseAltSpring = false, double defaultMass = 2.0, double defaultK = 0.0, double defaultDamp = 0.0, double defaultPointDamp = 1.0, double defaultPointResistence = 2.5, dtkDouble3 defaultGravityAccel = dtkDouble3( 0,0,0 ) );

    protected:
        //四面体网格
		dtkStaticTetraMesh::Ptr mTetraMesh; /**< 四面体弹簧网格 */


        bool mFullUseAltSpring;

        dtkPoints::Ptr mOriPointsPtr; /**< 点集 */

        double mAltitudeStiff;  /**< 弹簧刚性系数，弹性系数 */ 

        double mAltitudeDamp;  /**< 弹簧阻尼 */

		std::vector< std::vector< double > > mOriLengths; // numberOfTetra * 7  每个四面体的高及对边的距离

		std::map< dtkID2, size_t > mSpringCount; /**< 弹簧数 */
		
		/*
        dtkT3<T> mAltitudePoint1;
        dtkT3<T> mAltitudePoint2;
        dtkT3<T> mForceA;
        dtkT3<T> mForceB;
        dtkT3<T> mForceC;
        dtkT3<T> mForceD;
        */
        const static dtkID tetra_order[7][4]; 
	};
}

#endif
