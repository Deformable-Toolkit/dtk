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

        bool PreUpdate(double timeslice, ItrMethod method = Euler, dtkID iteration = 0);

        void DeleteTetra( dtkID i );

		void AddTetra( dtkID i );
		
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
		dtkStaticTetraMesh::Ptr mTetraMesh;

        bool mFullUseAltSpring;

        dtkPoints::Ptr mOriPointsPtr;

        double mAltitudeStiff;  

        double mAltitudeDamp;  

		std::vector< std::vector< double > > mOriLengths; // numberOfTetra * 7

		std::map< dtkID2, size_t > mSpringCount;
		
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
