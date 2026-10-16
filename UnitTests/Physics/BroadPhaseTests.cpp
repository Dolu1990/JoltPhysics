// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuadTree.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseBruteForce.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include "Layers.h"
#include <algorithm>
#include <future>
#include <thread>

template <class BroadPhaseType>
static void TestSphereQueryBatch()
{
	BPLayerInterfaceImpl layers;
	BodyManager body_manager;
	body_manager.Init(3, 0, layers);
	BroadPhaseType broadphase;
	broadphase.Init(&body_manager, layers);
	BodyID ids[3];
	for (uint i = 0; i < 3; ++i)
	{
		BodyCreationSettings settings(new BoxShape(Vec3::sOne()), RVec3(3.0 * i, 0, 0), Quat::sIdentity(), EMotionType::Static, i == 0? Layers::NON_MOVING : Layers::MOVING);
		Body *body = body_manager.AllocateBody(settings);
		REQUIRE(body != nullptr);
		REQUIRE(body_manager.AddBody(body));
		ids[i] = body->GetID();
	}
	BroadPhase::AddState add_state = broadphase.AddBodiesPrepare(ids, 3);
	broadphase.AddBodiesFinalize(ids, 3, add_state);
	const BroadPhaseQuery &query = broadphase;

	auto sort_hits = [](auto &hits) { sort(hits.begin(), hits.end(), [](BodyID a, BodyID b) { return a.GetIndexAndSequenceNumber() < b.GetIndexAndSequenceNumber(); }); };
	auto check_queries = [&](const BroadPhaseLayerFilter &bp_filter, const ObjectLayerFilter &object_filter)
	{
		Array<BodyID> expected[4];
		for (uint i = 0; i < 4; ++i)
		{
			AllHitCollisionCollector<CollideShapeBodyCollector> collector;
			query.CollideSphere(Vec3(3.0f * i, 0, 0), 1.25f, collector, bp_filter, object_filter);
			expected[i] = collector.mHits;
			sort_hits(expected[i]);
		}

		auto batch = query.BeginQueryBatch();
		for (uint repeat = 0; repeat < 3; ++repeat)
			for (uint i = 0; i < 4; ++i)
			{
				AllHitCollisionCollector<CollideShapeBodyCollector> collector;
				batch.CollideSphere(Vec3(3.0f * i, 0, 0), 1.25f, collector, bp_filter, object_filter);
				sort_hits(collector.mHits);
				CHECK(collector.mHits == expected[i]);

				struct FirstHitCollector : CollideShapeBodyCollector
				{
					virtual void AddHit(const BodyID &) override { ++mCount; ForceEarlyOut(); }
					uint mCount = 0;
				} first_hit;
				batch.CollideSphere(Vec3(3.0f * i, 0, 0), 1.25f, first_hit, bp_filter, object_filter);
				CHECK(first_hit.mCount == (expected[i].empty()? 0 : 1));
			}
	};

	for (uint pass = 0; pass < 2; ++pass)
	{
		check_queries(BroadPhaseLayerFilter(), ObjectLayerFilter());
		check_queries(SpecifiedBroadPhaseLayerFilter(BroadPhaseLayers::NON_MOVING), ObjectLayerFilter());
		check_queries(BroadPhaseLayerFilter(), SpecifiedObjectLayerFilter(Layers::MOVING));
		check_queries(BroadPhaseLayerFilter(), SpecifiedObjectLayerFilter(Layers::UNUSED1));

		// Every batch has ended before rebuilding or reclaiming the tree.
		body_manager.GetBody(ids[0]).SetPositionAndRotationInternal(RVec3(9, 0, 0), Quat::sIdentity());
		broadphase.NotifyBodiesAABBChanged(ids, 1, true);
		broadphase.Optimize();
	}

	atomic<uint> hits { 0 };
	thread workers[4];
	for (auto &worker : workers)
		worker = thread([&]
		{
			auto batch = query.BeginQueryBatch();
			for (uint i = 0; i < 64; ++i)
			{
				AllHitCollisionCollector<CollideShapeBodyCollector> collector;
				batch.CollideSphere(Vec3::sZero(), 20.0f, collector);
				hits.fetch_add(uint(collector.mHits.size()));
			}
		});
	for (auto &worker : workers)
		worker.join();
	CHECK(hits.load() == 4 * 64 * 3);
}

TEST_SUITE("BroadPhaseTests")
{
	TEST_CASE("TestBroadPhaseSphereQueryBatch")
	{
		TestSphereQueryBatch<BroadPhaseQuadTree>();
	}

	TEST_CASE("TestBroadPhaseSphereQueryBatchFallback")
	{
		TestSphereQueryBatch<BroadPhaseBruteForce>();
	}

	TEST_CASE("TestBroadPhaseQueryBatchDefersReclamation")
	{
		BPLayerInterfaceImpl layers;
		BodyManager body_manager;
		body_manager.Init(1, 0, layers);
		BroadPhaseQuadTree broadphase;
		broadphase.Init(&body_manager, layers);
		BodyCreationSettings settings(new BoxShape(Vec3::sOne()), RVec3::sZero(), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING);
		Body *body = body_manager.AllocateBody(settings);
		REQUIRE(body != nullptr);
		REQUIRE(body_manager.AddBody(body));
		BodyID id = body->GetID();
		BroadPhase::AddState add_state = broadphase.AddBodiesPrepare(&id, 1);
		broadphase.AddBodiesFinalize(&id, 1, add_state);

		promise<void> started, completed;
		auto started_future = started.get_future();
		auto completed_future = completed.get_future();
		thread cleanup;
		{
			auto batch = broadphase.BeginQueryBatch();
			// Publish the next tree while the batch still pins the previous query epoch.
			broadphase.LockModifications();
			auto state = broadphase.UpdatePrepare();
			broadphase.UpdateFinalize(state);
			broadphase.UnlockModifications();
			cleanup = thread([&]
			{
				started.set_value();
				broadphase.FrameSync();
				completed.set_value();
			});
			started_future.wait();
			CHECK(completed_future.wait_for(chrono::milliseconds(20)) == future_status::timeout);
			AllHitCollisionCollector<CollideShapeBodyCollector> collector;
			batch.CollideSphere(Vec3::sZero(), 1.0f, collector);
			CHECK(collector.mHits.size() == 1);
		}
		CHECK(completed_future.wait_for(chrono::seconds(5)) == future_status::ready);
		cleanup.join();
	}

	TEST_CASE("TestBroadPhaseOptimize")
	{
		BPLayerInterfaceImpl broad_phase_layer_interface;

		// Create body manager
		BodyManager body_manager;
		body_manager.Init(1, 0, broad_phase_layer_interface);

		// Create quad tree
		BroadPhaseQuadTree broadphase;
		broadphase.Init(&body_manager, broad_phase_layer_interface);

		// Create a box
		BodyCreationSettings settings(new BoxShape(Vec3::sOne()), RVec3::sZero(), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING);
		Body &body = *body_manager.AllocateBody(settings);
		body_manager.AddBody(&body);

		// Add it to the broadphase
		BodyID id = body.GetID();
		BroadPhase::AddState add_state = broadphase.AddBodiesPrepare(&id, 1);
		broadphase.AddBodiesFinalize(&id, 1, add_state);

		// Test that we hit the box at its current location and not where we're going to move it to
		AllHitCollisionCollector<RayCastBodyCollector> collector;
		broadphase.CastRay({ Vec3(0, 2, 0), Vec3(0, -2, 0) }, collector, BroadPhaseLayerFilter(), ObjectLayerFilter());
		CHECK(collector.mHits.size() == 1);
		CHECK(collector.mHits[0].mBodyID == id);
		CHECK_APPROX_EQUAL(collector.mHits[0].mFraction, 0.5f);
		collector.Reset();
		broadphase.CastRay({ Vec3(2, 2, 0), Vec3(0, -2, 0) }, collector, BroadPhaseLayerFilter(), ObjectLayerFilter());
		CHECK(collector.mHits.empty());
		broadphase.CastRay({ Vec3(4, 2, 0), Vec3(0, -2, 0) }, collector, BroadPhaseLayerFilter(), ObjectLayerFilter());
		CHECK(collector.mHits.empty());

		// Move the body
		body.SetPositionAndRotationInternal(RVec3(2, 0, 0), Quat::sIdentity());
		broadphase.NotifyBodiesAABBChanged(&id, 1, true);

		// Test that we hit the box at its previous and current location
		broadphase.CastRay({ Vec3(0, 2, 0), Vec3(0, -2, 0) }, collector, BroadPhaseLayerFilter(), ObjectLayerFilter());
		CHECK(collector.mHits.size() == 1);
		CHECK(collector.mHits[0].mBodyID == id);
		CHECK_APPROX_EQUAL(collector.mHits[0].mFraction, 0.5f);
		collector.Reset();
		broadphase.CastRay({ Vec3(2, 2, 0), Vec3(0, -2, 0) }, collector, BroadPhaseLayerFilter(), ObjectLayerFilter());
		CHECK(collector.mHits.size() == 1);
		CHECK(collector.mHits[0].mBodyID == id);
		CHECK_APPROX_EQUAL(collector.mHits[0].mFraction, 0.5f);
		collector.Reset();
		broadphase.CastRay({ Vec3(4, 2, 0), Vec3(0, -2, 0) }, collector, BroadPhaseLayerFilter(), ObjectLayerFilter());
		CHECK(collector.mHits.empty());

		// Optimize the broadphase
		broadphase.Optimize();

		// Test that we hit the box only at the new location
		broadphase.CastRay({ Vec3(0, 2, 0), Vec3(0, -2, 0) }, collector, BroadPhaseLayerFilter(), ObjectLayerFilter());
		CHECK(collector.mHits.empty());
		broadphase.CastRay({ Vec3(2, 2, 0), Vec3(0, -2, 0) }, collector, BroadPhaseLayerFilter(), ObjectLayerFilter());
		CHECK(collector.mHits.size() == 1);
		CHECK(collector.mHits[0].mBodyID == id);
		CHECK_APPROX_EQUAL(collector.mHits[0].mFraction, 0.5f);
		collector.Reset();
		broadphase.CastRay({ Vec3(4, 2, 0), Vec3(0, -2, 0) }, collector, BroadPhaseLayerFilter(), ObjectLayerFilter());
		CHECK(collector.mHits.empty());

		// Move the body again (so that for the next optimize we'll have to discard a tree)
		body.SetPositionAndRotationInternal(RVec3(4, 0, 0), Quat::sIdentity());
		broadphase.NotifyBodiesAABBChanged(&id, 1, true);

		// Test that we hit the box at its previous and current location
		broadphase.CastRay({ Vec3(0, 2, 0), Vec3(0, -2, 0) }, collector, BroadPhaseLayerFilter(), ObjectLayerFilter());
		CHECK(collector.mHits.empty());
		broadphase.CastRay({ Vec3(2, 2, 0), Vec3(0, -2, 0) }, collector, BroadPhaseLayerFilter(), ObjectLayerFilter());
		CHECK(collector.mHits.size() == 1);
		CHECK(collector.mHits[0].mBodyID == id);
		CHECK_APPROX_EQUAL(collector.mHits[0].mFraction, 0.5f);
		collector.Reset();
		broadphase.CastRay({ Vec3(4, 2, 0), Vec3(0, -2, 0) }, collector, BroadPhaseLayerFilter(), ObjectLayerFilter());
		CHECK(collector.mHits.size() == 1);
		CHECK(collector.mHits[0].mBodyID == id);
		CHECK_APPROX_EQUAL(collector.mHits[0].mFraction, 0.5f);
		collector.Reset();

		// Optimize the broadphase (this will internally have to discard a tree)
		broadphase.Optimize();

		// Test that we hit the box only at the new location
		broadphase.CastRay({ Vec3(0, 2, 0), Vec3(0, -2, 0) }, collector, BroadPhaseLayerFilter(), ObjectLayerFilter());
		CHECK(collector.mHits.empty());
		broadphase.CastRay({ Vec3(2, 2, 0), Vec3(0, -2, 0) }, collector, BroadPhaseLayerFilter(), ObjectLayerFilter());
		CHECK(collector.mHits.empty());
		broadphase.CastRay({ Vec3(4, 2, 0), Vec3(0, -2, 0) }, collector, BroadPhaseLayerFilter(), ObjectLayerFilter());
		CHECK(collector.mHits.size() == 1);
		CHECK(collector.mHits[0].mBodyID == id);
		CHECK_APPROX_EQUAL(collector.mHits[0].mFraction, 0.5f);
		collector.Reset();
	}
}
