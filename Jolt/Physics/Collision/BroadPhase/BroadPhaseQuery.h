// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/CollisionCollector.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Core/NonCopyable.h>
#include <Jolt/Core/Mutex.h>

JPH_NAMESPACE_BEGIN

struct RayCast;
class BroadPhaseCastResult;
class AABox;
class OrientedBox;
struct AABoxCast;

// Various collector configurations
using RayCastBodyCollector = CollisionCollector<BroadPhaseCastResult, CollisionCollectorTraitsCastRay>;
using CastShapeBodyCollector = CollisionCollector<BroadPhaseCastResult, CollisionCollectorTraitsCastShape>;
using CollideShapeBodyCollector = CollisionCollector<BodyID, CollisionCollectorTraitsCollideShape>;

/// Interface to the broadphase that can perform collision queries. These queries will only test the bounding box of the body to quickly determine a potential set of colliding bodies.
/// The shapes of the bodies are not tested, if you want this then you should use the NarrowPhaseQuery interface.
class JPH_EXPORT BroadPhaseQuery : public NonCopyable
{
public:
	/// A sequence of queries sharing one broadphase read lock when supported by the implementation.
	/// Create, use and destroy the batch on the same thread, and keep the BroadPhaseQuery alive throughout.
	/// Do not nest batches or call ordinary queries on the same broadphase from this thread while holding a batch.
	/// A batch does not lock bodies or freeze their state. Keep it short: it can delay broadphase tree reclamation.
	/// Release all batches before updating or optimizing the same physics system from the calling thread.
	class QueryBatch : public NonCopyable
	{
	public:
							~QueryBatch()
		{
			JPH_ASSERT(mThreadID == std::this_thread::get_id());
		}

		/// Get bodies intersecting with a sphere, reusing this batch's broadphase read lock.
		void				CollideSphere(Vec3Arg inCenter, float inRadius, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter = { }, const ObjectLayerFilter &inObjectLayerFilter = { }) const
		{
			JPH_ASSERT(mThreadID == std::this_thread::get_id());
			mQuery.CollideSphereNoLock(inCenter, inRadius, ioCollector, inBroadPhaseLayerFilter, inObjectLayerFilter);
		}

	private:
		friend class BroadPhaseQuery;

		explicit			QueryBatch(const BroadPhaseQuery &inQuery) : mQuery(inQuery)
		{
			if (SharedMutex *mutex = inQuery.GetQueryMutex())
				mLock = shared_lock<SharedMutex>(*mutex);
		}

		const BroadPhaseQuery &mQuery;
		shared_lock<SharedMutex> mLock;
		JPH_IF_ENABLE_ASSERTS(std::thread::id mThreadID = std::this_thread::get_id();)
	};

	/// Begin a scoped sequence of sphere queries. Implementations without batch support use ordinary queries.
	QueryBatch			BeginQueryBatch() const									{ return QueryBatch(*this); }

	/// Virtual destructor
	virtual				~BroadPhaseQuery() = default;

	/// Cast a ray and add any hits to ioCollector
	virtual void		CastRay(const RayCast &inRay, RayCastBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter = { }, const ObjectLayerFilter &inObjectLayerFilter = { }) const = 0;

	/// Get bodies intersecting with inBox and any hits to ioCollector
	virtual void		CollideAABox(const AABox &inBox, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter = { }, const ObjectLayerFilter &inObjectLayerFilter = { }) const = 0;

	/// Get bodies intersecting with a sphere and any hits to ioCollector
	virtual void		CollideSphere(Vec3Arg inCenter, float inRadius, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter = { }, const ObjectLayerFilter &inObjectLayerFilter = { }) const = 0;

	/// Get bodies intersecting with a point and any hits to ioCollector
	virtual void		CollidePoint(Vec3Arg inPoint, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter = { }, const ObjectLayerFilter &inObjectLayerFilter = { }) const = 0;

	/// Get bodies intersecting with an oriented box and any hits to ioCollector
	virtual void		CollideOrientedBox(const OrientedBox &inBox, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter = { }, const ObjectLayerFilter &inObjectLayerFilter = { }) const = 0;

	/// Cast a box and add any hits to ioCollector
	virtual void		CastAABox(const AABoxCast &inBox, CastShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter = { }, const ObjectLayerFilter &inObjectLayerFilter = { }) const = 0;

protected:
	/// Override together with CollideSphereNoLock to support locking once per batch.
	/// The selected mutex must protect queries for the complete batch lifetime, including tree epoch changes.
	virtual SharedMutex *GetQueryMutex() const									{ return nullptr; }

	/// Only QueryBatch and the implementation's locked wrapper may call the unlocked traversal.
	/// The default implementation preserves normal locking for broadphases without batch support.
	virtual void		CollideSphereNoLock(Vec3Arg inCenter, float inRadius, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const
	{
		CollideSphere(inCenter, inRadius, ioCollector, inBroadPhaseLayerFilter, inObjectLayerFilter);
	}
};

JPH_NAMESPACE_END
