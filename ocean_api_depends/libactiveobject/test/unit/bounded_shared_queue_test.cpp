// Test the active_idle class

#include "gtest/gtest.h"

#include "active_object/bounded_shared_queue.h"

using namespace active_object;

TEST( BoundedSharedQueue, TestConstructorDestructor ) {
	bounded_shared_queue<int, 5> queue;

	ASSERT_EQ( queue.size(), 0 );
}

TEST( BoundedSharedQueue, TestPushPop ) {
	bounded_shared_queue<int, 5> queue;

	ASSERT_EQ( queue.size(), 0 );
	auto result = queue.push( 0 );

	ASSERT_EQ( queue.size(), 1 );
	ASSERT_EQ( result, 0 );       // No data was dropped to push this value

	int val;
	auto popped = queue.try_and_pop(val);
	ASSERT_TRUE( popped );
	ASSERT_EQ( val, 0 );
}

TEST( BoundedSharedQueue, TestBoundedPushPop ) {
	const int QueueSize = 5;
	bounded_shared_queue<int, QueueSize> queue;

	ASSERT_EQ( queue.size(), 0 );

	const int Offset = 127;   // Use non-zero values just in case

	for(  int i = 0; i < QueueSize; ++i ) {
		auto result = queue.push( i+Offset );

		ASSERT_EQ( queue.size(), i+1 );
		ASSERT_EQ( result, 0 );
	}

	ASSERT_EQ( queue.size(), QueueSize );

	// This pop should push a value (== Offset+0) off the stack
	{
		auto result = queue.push( Offset + QueueSize );

		ASSERT_EQ( queue.size(), QueueSize );
		ASSERT_EQ( result, 1 );
	}

	// Now everything is offset by 1
	for(  int i = 1; i <= QueueSize; ++i ) {
		int val;
		auto popped = queue.try_and_pop( val );

		ASSERT_TRUE( popped );
		ASSERT_EQ( queue.size(), QueueSize - i );
		ASSERT_EQ( val, Offset + i );
	}

}
