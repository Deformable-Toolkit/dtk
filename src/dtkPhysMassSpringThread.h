#ifndef DTK_PHYSMASSSPRINGTHREAD_H
#define DTK_PHYSMASSSPRINGTHREAD_H

#include <memory>
#include <boost/utility.hpp>
#include "dtkPhysTetraMassSpring.h"

namespace dtk
{
    class dtkPhysMassSpringThread : public dtkPhysTetraMassSpring
    {
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

		    typedef std::shared_ptr< dtkPhysMassSpringThread > Ptr;

		    static Ptr New(double interval, int length, dtkT3<double> firstPos, Orientation ori, double mass,
				double edgeStiff, double bendStiff,double torsionStiff, 
				double edgeDamp, double extraEdgeDamp, double bendDamp, double torsionDamp)
		    {
			    return Ptr(new dtkPhysMassSpringThread(interval, length, firstPos, ori, mass, edgeStiff,
					bendStiff, torsionStiff, edgeDamp, extraEdgeDamp, bendDamp, torsionDamp));
		    };

        public:
	        ~dtkPhysMassSpringThread();

			bool ApplyImpulse( double timeslice );

	        void constructThreadMesh();

	        void constructTetraMesh();

	        void addEdgeSpring();

            void addExtraEdgeSpring();

	        void addBendSpring();

	        void addTorsionSpring();

            dtk::dtkStaticTetraMesh::Ptr getTetraMesh() 
            { 
                return mTetraMeshPtr; 
            }

            size_t GetNumberOfSegments() { 
                return mLength; 
            }

			double GetInterval() { 
				return mInterval; 
			}

            void ControlEndPropagate(double timeslice, int range = 30);

            void ImpulsePropagate(dtkT3<double> impulse, dtkID pointID, dtkID ori, size_t range = 30);

        private:
	        dtkPhysMassSpringThread(double interval, int length, dtk::dtkT3<double> firstPos, Orientation ori, double mass,
				double edgeStiff, double bendStiff,double torsionStiff, 
				double edgeDamp, double extraEdgeDamp, double bendDamp, double torsionDamp);

	        dtk::dtkStaticTetraMesh::Ptr mTetraMeshPtr;

	        double mEdgeStiff;
    
            double mExtraEdgeStiff;
	
            double mBendStiff;
	
            double mTorsionStiff;

            double mEdgeDamp;
    
            double mExtraEdgeDamp;
	
            double mBendDamp;
	
            double mTorsionDamp;
	
	        size_t mLength;
	
            dtkT3<double> mFirstPos;
	
            double mInterval;
    
            double mRotateInterval;

            dtk::dtkID mOrientation;
    };
}

#endif
