#ifndef dtkPHYSSPRING_H
#define dtkPHYSSPRING_H

#include "dtkPhysMassPoint.h"
#include "dtkIDTypes.h"

namespace dtk
{
	class dtkPhysSpring
	{
	public:
		dtkPhysSpring(dtkPhysMassPoint* p1, dtkPhysMassPoint* p2, 
			const double& stiff = 5, const double& damp = 0);

		dtkPhysMassPoint* GetFirstVertex() { return mVerteces[0]; }
		dtkPhysMassPoint* GetSecondVertex() { return mVerteces[1]; }

		bool Update(double timeslice, ItrMethod method = Euler, dtkID iteration = 0, bool limitDeformation = false);
        void SetStiffness(double newStiffness) { mStiffness = newStiffness; } 
        void SetDamp(double newDamp) { mDamp = newDamp; }
        double GetOriLength() { return mOriLength; }
        double GetStiffness() { return mStiffness; }
        double GetDamp() { return mDamp; }

	public:
		virtual ~dtkPhysSpring();

	private:
		dtkPhysMassPoint* mVerteces[2];

		double mOriLength;
		double mStiffness;
		double mDamp;

	};
}

#endif
