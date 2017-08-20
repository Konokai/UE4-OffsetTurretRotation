#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "TurretRotationFunctionLibrary.generated.h"

/** This function library handles rotation for turrets with an "offset" aim joint. */
UCLASS()
class TURRETROTATION_API UTurretRotationFunctionLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Forces the construction script to run for a given Actor.
	 * We use this to make sure that turrets update whenever targets are moved inside the editor.
	 *
	 * @param MyActor	The actor to update
	 */
	UFUNCTION( BlueprintCallable )
	static void ForceExecuteConstructionScript( AActor* MyActor );

	/**
	 * This is the "complete" turret calculation function. 
	 * If you're looking for the "final" function to get your own turret working in your own project, then use this one.
	 * This function will calculate the rotation needed for the AimJoint to aim the turret's barrel towards the given target.
	 *
	 * @param ActorWorldTransform		This is included so that an Actor's Scale/Rotation/Translation are handled during the rotation calculation.
	 * @param Actor_To_AimJoint			The vector from the Actor's location to the AimJoint's location (when the Actor is not Rotated/Scaled).
	 * @param AimJoint_To_BarrelStart	The vector from the AimJoint to the BarrelStart (when the Actor is not Rotated/Scaled).
	 * @param BarrelStart_To_BarrelEnd	The vector for the BarrelStart to the BarrelEnd (when the Actor is not Rotated/Scaled).
	 * @param Out_AimJointRotation		OUT - The new rotation for the AimJoint (relative to the Actor).
	 */
	UFUNCTION( BlueprintPure )
	static void CalculateTurretRotation_ForActor(
		const FTransform& ActorWorldTransform,
		const FVector& Actor_To_AimJoint,
		const FVector& AimJoint_To_BarrelStart,
		const FVector& BarrelStart_To_BarrelEnd,
		const FVector& TargetWorldLocation,
		FRotator& Out_AimJointRotation );

	/** 
	 * Calculates turret rotation based on the AimJoint's world transform, and the BarrelStart/BarrelEnd/TargetWorldLocation.
	 * It is assumed that the Actor's transform was already handled in CalculateTurretRotation_ForActor in order to calculate the AimJointWorldSpace transform.
	 * If that's not the case, then this function may only work if the Actor has no Rotation/Scale applied to it.
	 * 
	 * @param AimJointWorldSpaceTransform	Transform that represents the AimJoint in world space.
	 * @param AimJoint_To_BarrelStart		The vector from the AimJoint to the BarrelStart. This is expected to be scaled by the Actor's scale in CalculateTurretRotation_ForActor.
	 * @param BarrelStart_To_BarrelEnd		The vector from the BarrelStart to the BarrlEnd. This is expected to be scaled by the Actor's scale in  CalculateTurretRotation_ForActor.
	 * @param TargetWorldLocation			The target's location in world space.
	 * @param Out_AimJointRotation			OUT - The new rotation for the AimJoint (relative to the Actor).
	 */
	UFUNCTION( BlueprintPure )
	static void CalculateTurretRotation_ForAimJoint(
		const FTransform& AimJointWorldSpaceTransform,
		const FVector AimJoint_To_BarrelStart,
		const FVector BarrelStart_To_BarrelEnd,
		const FVector TargetWorldLocation,
		FRotator& Out_AimJointRotation );

	/**
	 * In Unreal, Z is "up", and the "X-Y" plane makes up the horizontal plane.
	 * This calculates the yaw, or the angle across the "X-Y" plane, for the AimJoint to rotate until it is aligned with the target location. 
	 *
	 * @param AimJointLocation	Location of the AimJoint.  
	 * @param TargetLocation	Location of the target.
	 * @return Returns the yaw needed to align the AimJoint (and the turret) with the target location.
	 */
	UFUNCTION( BlueprintPure )
	static float CalculateTurretYaw( const FVector& AimJointLocation, const FVector& TargetLocation );

	/**
	 * This function assumes that the AimJoint, BarrelStart, BarrelEnd, and TargetLocation are all aligned on the "X-Z" plane.
	 * It calculates the pitch, or the angle on the "X-Z" plane, for the AimJoint to rotate so the turret points to the TargetLocation.
	 * 
	 * @param AimJointLocation	Location of the AimJoint.
	 * @param BarrelStart		Location of the BarrelStart.
	 * @param BarrelEnd			Location of the BarrelEnd.
	 * @param TargetLocation	Location of the Target.
	 * @return Returns the pitch, or the angle on the "X-Z" plane, for the AimJoint to rotate so the turret points to the TargetLocation.
	 */
	UFUNCTION( BlueprintPure )
	static float CalculateTurretPitch(
		const FVector& AimJointLocation,
		const FVector& BarrelStartLocation,
		const FVector& BarrelEndLocation,
		const FVector& TargetLocation );

private:

	/**
	 * The correct pitch to calculate in CalculateTurretPitch is undefined if the Target is too close to the AimJoint.
	 * If the Target is too close to the AimJoint, then it is technically impossible for the turret to point at the Target.
	 * If an invalid Target Location is detected, then this function will return a new Location that is far enough from the AimJoint for 
	 * calculations to continue. This will result in behavior that "makes sense" for the invalid case, even though the turret won't end up 
	 * pointing at the Target.
	 *
	 * @param AimJointLocation2D	Location of the AimJoint.
	 * @param BarrelStartLocation2D	Location of the BarrelStart.
	 * @param BarrelEndLocation2D	Location of the BarrelEnd.
	 * @param TargetLocation2D		Location of the Target.
	 * @return Returns a "valid" Target Location that should result in behavior that "makes sense" if the Target is too close to the AimJoint.
	 */
	static FVector2D CalculateNearestValidTargetLocation2D(
		const FVector2D& AimJointLocation2D,
		const FVector2D& BarrelStartLocation2D,
		const FVector2D& BarrelEndLocation2D,
		const FVector2D& TargetLocation2D );

	/** 
	 * For CalculateNearestValidTargetLocation2D, find the Out_BarrelRayDistance so that the ScaledBarrelEnd can be found.
	 *
	 * @param AimJointLocation2D	Location of the AimJoint.
	 * @param BarrelStartLocation2D	Location of the BarrelStart.
	 * @param BarrelEndLocation2D	Location of the BarrelEnd.
	 * @param TargetLocation2D		Location of the Target.
	 * @param Out_BarrelRayDistance	OUT - The found BarrelRayDistance.
	 * @return Returns true if a valid BarrelRayDistance was found, otherwise false.
	 */
	static bool CalculateBarrelRayDistance( const FVector2D& AimJointLocation2D,
									        const FVector2D& BarrelStartLocation2D,
									        const FVector2D& BarrelEndLocation2D,
									        const FVector2D& TargetLocation2D,
									        float& Out_BarrelRayDistance );

	/** 
	 * Calculates the quadratic coefficients for CalculateBarrelRayDistance
	 * 
	 * @param AimJointLocation2D	Location of the AimJoint.
	 * @param BarrelStartLocation2D	Location of the BarrelStart.
	 * @param BarrelEndLocation2D	Location of the BarrelEnd.
	 * @param TargetLocation2D		Location of the Target.
	 * @param Out_A					OUT - Value of coefficient "a"
	 * @param Out_A					OUT - Value of coefficient "b"
	 * @param Out_C					OUT - Value of coefficient "c"
	 */
	static void CalculateQuadraticCoefficients(
		const FVector2D& AimJointLocation2D,
		const FVector2D& BarrelStartLocation2D,
		const FVector2D& BarrelEndLocation2D,
		const FVector2D& TargetLocation2D,
		float& Out_A,
		float& Out_B,
		float& Out_C );

	/** 
	 * Calculates the quadratic roots for CalculateBarrelRayDistance
	 *
	 * @param A			Quadratic coefficient "a"
	 * @param B			Quadratic coefficient "b"
	 * @param C			Quadratic coefficient "c"
	 * @param Out_X1	OUT - Quadratic coefficient "x1"
	 * @param Out_X2	OUT - Quadratic coefficient "x2"
	 * @return Returns true if the roots could be found, otherwise false.
	 */
	static bool CalculateQuadraticRoots( float A, float B, float C, float& Out_X1, float& Out_X2 );

	/**
	 * Given two ray distances, this selects the best one (the one in front of BarrelStart).
	 *
	 * @param FirstDistance		First ray distance
	 * @param SecondDistance	Second ray distance
	 * @return Returns the best ray distance (the one in front of BarrelStart).
	 */
	static float SelectBestRayDistance( float FirstDistance, float SecondDistance );


	/** 
	 * Calculates the angle to rotate from the FirstVector to the SecondVector.  Also handles whether to rotate clockwise or counterclockwise.
	 *
	 * @param FirstVector	Arbitrary 2D vector.
	 * @param SecondVector	Arbitrary 2D vector.
	 * @return Returns the angle needed to rotate the FirstVector to meet the SecondVector.
	 */
	static float CalculateAngleToRotateFromFirstVectorToSecondVector( FVector2D FirstVector, FVector2D SecondVector );

	/** 
	 * Calculates the angle between the two given normalized 2D vectors.
	 *
	 * @param FirstVector	Normalized 2D vector.
	 * @param SecondVector	Normalized 2D vector.
	 * @return Returns the angle between the two given normalized vectors.
	 */
	static float CalculateAngleBetweenNormalizedVectors( FVector2D FirstVector_Normalized, FVector2D SecondVector_Normalized );

	/**
	 * Decides if the FirstVector should be rotated counterclockwise to meet the SecondVector (implies that a counterclockwise rotation
	 * is shorter than a clockwise one).
	 * 
	 * @param FirstVector	Arbitrary 2D vector.
	 * @param SecondVector	Arbitrary 2D vector.
	 * @return Returns true if the FirstVector should be rotated counterclockwise to meet the SecondVector, otherwise false.
	 */
	static bool ShouldTurnCounterClockwiseToMeet( FVector2D FirstVector, FVector2D SecondVector );
};
