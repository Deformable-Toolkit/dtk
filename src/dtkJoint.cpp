/**
 * @Author: tom: https://github.com/TOMsworkspace 
 * @Date: 2021-08-05 21:21:18 
 * @Last Modified by: tom: https://github.com/TOMsworkspace
 * @Last Modified time: 2021-08-06 17:34:49
 */

#include "dtkJoint.h"

namespace dtk {

    dtkJoint::dtkJoint(dtkRigidBody::ptr a, dtkRigidBody::ptr b) : mBodyA(a), mBodyB(b) {}

    dtkRigidBody::ptr dtkJoint::get_a() const {
        return mBodyA.lock();
    }

    void dtkJoint::set_a(dtkRigidBody::ptr a) {
        mBodyA = a;
    }

    dtkRigidBody::ptr dtkJoint::get_b() const {
        return mBodyB.lock();
    }

    void dtkJoint::set_b(dtkRigidBody::ptr b) {
        mBodyB = b;
    }

// ---------------------------------------------------
// dtkRevoluteJoint

    dtkRevoluteJoint::dtkRevoluteJoint(dtkRigidBody::ptr a, dtkRigidBody::ptr b, const dtkDouble2 &anchor)
        : dtkJoint(a, b), mAnchor(anchor) {
        mLocalAnchorA = glm::transpose(2, 2, float, a->get_rotation()) * (mAnchor - a->local_to_world(a->get_centroid()));
        mLocalAnchorB = glm::transpose(2, 2, float, b->get_rotation()) * (mAnchor - b->local_to_world(b->get_centroid()));
    }

    void dtkRevoluteJoint::pre_step(double dt) {
        static const double kBiasFactor = 0.2;
        auto a = mBodyA.lock();
        auto b = mBodyB.lock();
        mRotateA = a->get_rotation() * mLocalAnchorA;
        mRotateB = b->get_rotation() * mLocalAnchorB;
        dtkMatrix22 k = (a->get_inv_mass() + b->get_inv_mass()) * dtkMatrix22(1.0) +
                 a->get_inv_inertia() * dtkMatrix22(mRotateA.y*mRotateA.y, -mRotateA.y*mRotateA.x, -mRotateA.y*mRotateA.x, mRotateA.x*mRotateA.x) +
                 b->get_inv_inertia() * dtkMatrix22(mRotateB.y*mRotateB.y, -mRotateB.y*mRotateB.x, -mRotateB.y*mRotateB.x, mRotateB.x*mRotateB.x);
        mMassAll = glm::inverse(2, 2, float, k);// get k.inverse();
        mBias = -kBiasFactor / dt *(b->local_to_world(b->get_centroid())
            + mRotateB - a->local_to_world(a->get_centroid()) - mRotateA);
        a->update_impulse(-mMomentum, mRotateA);
        b->update_impulse(mMomentum, mRotateB);
    }

    void dtkRevoluteJoint::update_impulse() {
        auto a = mBodyA.lock();
        auto b = mBodyB.lock();
        auto dv = (b->get_velocity() + cross(b->get_angular_velocity(), mRotateB)) -
                  (a->get_velocity() + cross(a->get_angular_velocity(), mRotateA));
        auto p = mMassAll * (-1.0 * dv + mBias);
        a->update_impulse(-p, mRotateA);
        b->update_impulse(p, mRotateB);
        mMomentum += p;
    }

    const dtkDouble2 &dtkRevoluteJoint::anchor() const { return mAnchor; }

    dtkDouble2 dtkRevoluteJoint::world_anchor_a() const {
        auto a = mBodyA.lock();
        return a->local_to_world(a->get_rotation() * mLocalAnchorA + a->get_centroid());
    }

    dtkDouble2 dtkRevoluteJoint::world_anchor_b() const {
        auto b = mBodyB.lock();
        return b->local_to_world(b->get_rotation() * mLocalAnchorB + b->get_centroid());
    }
}
