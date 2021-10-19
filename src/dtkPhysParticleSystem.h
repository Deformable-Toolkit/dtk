#ifndef DTK_PHYSPARTICLESYSTEM_H
#define DTK_PHYSPARTICLESYSTEM_H

#include <memory>
#include <boost/utility.hpp>
#include <vector>

#include "dtkPhysParticle.h"

namespace dtk
{
	class dtkPhysParticleSystem : public boost::noncopyable
	{
	public:
		typedef std::shared_ptr< dtkPhysParticleSystem > Ptr;

		static Ptr New( double particleRadius, double particleMass, double particleLifetime ) 
		{
			return Ptr( new dtkPhysParticleSystem( particleRadius, particleMass, particleLifetime ) );
		}

	public:
		virtual ~dtkPhysParticleSystem();

		const GK::Point3& GetPoint(dtkID id) const;
		void SetPoint( dtkID id, const GK::Point3& point ) { mParticles[id]->SetPoint( point ); }

		size_t GetNumberOfParticles() { return mParticles.size(); }
		const std::vector< dtkPhysParticle* >& GetParticles() const { return mParticles; }
		
		dtkID AddParticle( const GK::Point3& position, const double& lifetime, const double& mass = 1.0, const dtkT3<double>& vel = dtkT3<double>(0,0,0) );
		dtkPhysParticle* GetParticle( dtkID id ) { return mParticles[id]; }

		/**
		* @brief		更新粒子数量及状态
		* @param[in]	timeslice : 更新时间间隔
		* @note			更新粒子数量及状态
		* @return		
		*	true update successfully \n
		*	false update failure \n
		*/
		bool Update(double timeslice);

		void SetEmitPosition( const GK::Point3& emitPosition ) { mEmitPosition = emitPosition; }
		const GK::Point3& GetEmitPosition() const { return mEmitPosition; }

		void SetInitialVelocity( const dtkT3<double>& vel ) { mDefaultVelocity = vel; }
		const dtkT3<double>& GetInitialVelocity() const { return mDefaultVelocity; }

		void StartEmit();
		void StopEmit();
		bool IsEmitting() const { return mEmitting; }

		void SetForceAffector( dtkDouble3 force ) { mForceAffector = force; }
		const dtkDouble3& GetForceAffector() const { return mForceAffector; }
		void AddForceAffector( dtkDouble3 force );

		void SetForceDisturbance( dtkDouble3 force ) { mForceDisturbance = force; }
		const dtkDouble3& GetForceDisturbance() const { return mForceDisturbance; }

		void SetEmitRate( double emitRate ) { mEmitRate = emitRate; }
		double GetEmitRate() const { return mEmitRate; }

		void SetLifetime( double lifetime ) { mDefaultLifetime = lifetime; }
		double GetLifetime() const { return mDefaultLifetime; }

		void SetMass( double mass ) { mDefaultMass = mass; }
		double GetMass() const { return mDefaultMass; }

		double GetParticleRadius() const { return mDefaultParticleRadius; }

	private:
		dtkPhysParticleSystem( double particleRadius, double particleMass, double particleLifetime );
		std::vector< dtkPhysParticle* > mParticles;

		GK::Point3 mEmitPosition; /**< 发散位置 */
		bool mEmitting; /**< 发散状态 */
		double mDefaultParticleRadius; /**< 默认粒子半径 */
		double mDefaultMass; /**< 默认质量 */
		double mDefaultLifetime;  /**< 默认存活时间 */
		dtkDouble3 mDefaultVelocity; /**< 速度 */

		dtkDouble3 mForceAffector; /**< 附加力 */
		dtkDouble3 mForceDisturbance; /**< 干扰力 */

		double mEmitRate;  /**< 发散率 */
	};
}

#endif

