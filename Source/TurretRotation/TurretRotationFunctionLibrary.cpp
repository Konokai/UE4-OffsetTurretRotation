#include "TurretRotationFunctionLibrary.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"


void UTurretRotationFunctionLibrary::ForceExecuteConstructionScript( AActor* MyActor )
{
	if ( !MyActor )
	{
		return;
	}

	UWorld* MyWorld = MyActor->GetWorld();
	if ( !MyWorld || MyWorld->WorldType != EWorldType::Editor )
	{
		return;
	}

	MyActor->RerunConstructionScripts();
}

void UTurretRotationFunctionLibrary::CalculateTurretRotation_ForActor( 
	const FTransform& ActorWorldTransform, 
	const FVector& Actor_To_AimJoint, 
	const FVector& AimJoint_To_TurretBarrelStart, 
	const FVector& TurretBarrelStart_To_TurretBarrelEnd, 
	const FVector& TargetWorldLocation, 
	FRotator& Out_AimJointRotation )
{
	FTransform ActorWorldTransform_ScaleOnly = FTransform();
	ActorWorldTransform_ScaleOnly.SetScale3D( ActorWorldTransform.GetScale3D() );

	// The Actor's Rotation/Translation will not affect the relative positions of the BarrelStart/BarrelEnd locations, but Scale will!
	// Calculate the new BarrelStart/BarrelEnd relative locations based on the Actor's current scale.
	const FVector AimJoint_To_BarrelStart_Scaled = ActorWorldTransform_ScaleOnly.TransformPosition( AimJoint_To_TurretBarrelStart );
	const FVector BarrelStart_To_BarrelEnd_Scaled = ActorWorldTransform_ScaleOnly.TransformPosition( TurretBarrelStart_To_TurretBarrelEnd ); 

	// Using the Actor's world transform, find the AimJoint's world transform.
	const FTransform Actor_To_AimJointTransform = FTransform( Actor_To_AimJoint );
	const FTransform AimJointWorldTransform = Actor_To_AimJointTransform * ActorWorldTransform;

	// With these things done we no longer need the Actor's world transform, only the AimJoint's world transform.
	// So we can go ahead and calculate the rotation for the turret.
	CalculateTurretRotation_ForAimJoint( 
		AimJointWorldTransform,
		AimJoint_To_BarrelStart_Scaled,
		BarrelStart_To_BarrelEnd_Scaled,
		TargetWorldLocation, 
		/*out*/ Out_AimJointRotation );
}

void UTurretRotationFunctionLibrary::CalculateTurretRotation_ForAimJoint( 
	const FTransform& AimJointWorldTransform,
	const FVector AimJoint_To_BarrelStart, 
	const FVector BarrelStart_To_BarrelEnd, 
	const FVector TargetWorldLocation, 
	FRotator& Out_AimJointRotation )
{
	// How far away is the target relative to the AimJoint?  Where is it relative to the AimJoint?
	// To answer these questions we'll find the target's Location relative to the AimJoint.
	// We're removing Scale using AimJointWorldTransform_WithoutScale since we only care about Rotation/Translation when 
	// finding the target's location relative to the AimJoint.
	FTransform AimJointWorldTransform_WithoutScale = AimJointWorldTransform;
	AimJointWorldTransform_WithoutScale.SetScale3D( FVector::OneVector );
	const FTransform World_To_AimJoint_Transform = AimJointWorldTransform_WithoutScale.Inverse();

	// We have a bunch of different vectors in a bunch of different spaces.  
	// For example, TargetWorldLocation is in world space while AimJoint_To_BarrelStart and BarrelStart_To_BarrelEnd are relative translation vectors.
	// Since we're finding the rotation for the AimJoint... it's useful to find where all of these different parts in AimJoint space.
	const FVector AimJoint_InAimJointSpace = FVector::ZeroVector;
	const FVector BarrelStart_InAimJointSpace = AimJoint_To_BarrelStart;
	const FVector BarrelEnd_InAimJointSpace = BarrelStart_InAimJointSpace + BarrelStart_To_BarrelEnd;
	const FVector Target_InAimJointSpace = World_To_AimJoint_Transform.TransformPosition( TargetWorldLocation );
	
	// Now that everything is in the same space, it's relatively straightforward to go ahead and calculate the yaw needed for the AimJoint.
	// This NewYaw is important since it can be used to align the turret with the target.
	const float NewYaw = CalculateTurretYaw( AimJoint_InAimJointSpace, Target_InAimJointSpace );

	// Calculating the pitch is really a 2D problem.  Since that's the case, we need to make sure that the AimJoint, BarrelStart, BarrelEnd,
	// and the Target are all aligned on the same 2D plane.
	// The NewYaw we just calculated will rotate the turret to align with the target... so if we invert that, we have a rotator
	// that will instead align the target with the turret.
	const FRotator Align_Turret_With_Target_Rotator = FRotator(0.0f /*Pitch*/, NewYaw /*Yaw*/, 0.0f /*Roll*/ );
	const FRotator Align_Target_With_Turret_Rotator = Align_Turret_With_Target_Rotator.GetInverse();

	const FVector Target_InAimJointSpace_AlignedWithTurret = Align_Target_With_Turret_Rotator.RotateVector( Target_InAimJointSpace );

	// Now that we successfully used the NewYaw to align the target's location with the turret, we have everything on a nice 2D plane.
	// This gives us everything we need to go ahead and calculate the pitch.
	const float NewPitch = CalculateTurretPitch( 
		AimJoint_InAimJointSpace, 
		BarrelStart_InAimJointSpace,
		BarrelEnd_InAimJointSpace,
		Target_InAimJointSpace_AlignedWithTurret );

	// Set the results.
	Out_AimJointRotation = FRotator::ZeroRotator;
	Out_AimJointRotation.Yaw = NewYaw;
	Out_AimJointRotation.Pitch = NewPitch;
}

float UTurretRotationFunctionLibrary::CalculateTurretYaw( const FVector& AimJointLocation, const FVector& TargetLocation )
{
	const FVector AimJoint_To_Target = TargetLocation - AimJointLocation;

	// Atan2 will give us the angle (in radians) that corresponds to AimJoint_To_Target.
	// See https://en.wikipedia.org/wiki/Atan2
	const float AngleFromAimJointToTarget_Radians = FMath::Atan2( AimJoint_To_Target.Y, AimJoint_To_Target.X );
	const float AngleFromAimJointToTarget_Degrees = FMath::RadiansToDegrees( AngleFromAimJointToTarget_Radians );

	return AngleFromAimJointToTarget_Degrees;
}

float UTurretRotationFunctionLibrary::CalculateTurretPitch( 
	const FVector& AimJointLocation, 
	const FVector& BarrelStartLocation, 
	const FVector& BarrelEndLocation, 
	const FVector& TargetLocation )
{
	// Since we're assuming that all of these locations are already aligned on the "X-Z" plane, we know that this is really a 2D problem.
	// So go ahead and convert our inputs into 2D vectors.
	const FVector2D AimJointLocation2D = FVector2D( AimJointLocation.X, AimJointLocation.Z );
	const FVector2D BarrelStartLocation2D = FVector2D( BarrelStartLocation.X, BarrelStartLocation.Z );
	const FVector2D BarrelEndLocation2D = FVector2D( BarrelEndLocation.X, BarrelEndLocation.Z );
	FVector2D TargetLocation2D = FVector2D( TargetLocation.X, TargetLocation.Z );

	// Targets that are too close to the AimJoint are invalid, so, if that's the case, then get a "valid" location for the Target.
	TargetLocation2D = CalculateNearestValidTargetLocation2D( AimJointLocation2D, BarrelStartLocation2D, BarrelEndLocation2D, TargetLocation2D );

	// The pitch required to rotate the AimJoint changes depending on how far away the Target is from the AimJoint.	
	//
	// If (AimJoint_To_Target_Distance == AimJoint_To_BarrelEnd_Distance), then we can easily find the pitch.
	// Just find the angle between AimJoint_To_BarrelEnd and AimJoint_To_Target.
	//
	// If (AimJoint_To_Target_Distance != AimJoint_To_BarrelEnd_Distance), then we have to do some work.
	// We have to find the "ScaledBarrelEnd" such that (AimJoint_To_Target_Distance == AimJoint_To_ScaledBarrelEnd_Distance).
	// Then we can find the pitch by finding the angle between the AimJoint_To_ScaledBarrelEnd and AimJoint_To_Target.
	//
	// The "BarrelRay" is a normalized vector that points from the BarrelStart to the BarrelEnd.
	// The "ScaledBarrelEnd" sits somewhere along the BarrelRay.  
	// Find the "BarrelRayDistance", which is the distance from the BarrelStart to the ScaledBarrelEnd.
	float BarrelRayDistance = 0.0f;
	const bool bFoundRayDistance = CalculateBarrelRayDistance( AimJointLocation2D, BarrelStartLocation2D, BarrelEndLocation2D, TargetLocation2D, /*out*/ BarrelRayDistance );
	if ( !bFoundRayDistance )
	{
		// For any really weird cases (like where the AimJoint, the BarrelStart, the BarrelEnd, and the TargetLocation are all equal), just return 0.0f.
		return 0.0f;
	}

	// Now that we have the BarrelRayDistance, we can easily find the "ScaledBarrelEnd."
	const FVector2D BarrelRay = ( BarrelEndLocation2D - BarrelStartLocation2D ).GetSafeNormal();
	FVector2D ScaledBarrelEndLocation2D = BarrelStartLocation2D + ( BarrelRay * BarrelRayDistance );
	
	// The angle between these two vectors represents the pitch.
	const FVector2D AimJoint_To_ScaledBarrelEnd = ScaledBarrelEndLocation2D - AimJointLocation2D;
	const FVector2D AimJoint_To_Target = TargetLocation2D - AimJointLocation2D;

	// Calculate the resulting pitch.  This does a bit more than just calculate the angle - it also tells us if we need to rotate
	// clockwise or counter-clockwise to meet the target.
	const float Result = CalculateAngleToRotateFromFirstVectorToSecondVector( AimJoint_To_ScaledBarrelEnd, AimJoint_To_Target );

	return Result;
}

FVector2D UTurretRotationFunctionLibrary::CalculateNearestValidTargetLocation2D( 
	const FVector2D& AimJointLocation2D, 
	const FVector2D& BarrelStartLocation2D, 
	const FVector2D& BarrelEndLocation2D, 
	const FVector2D& TargetLocation2D )
{
	// If the Target is closer than both the BarrelStart/BarrelEnd, then the Target is invalid.

	const float AimJoint_To_BarrelStart_Distance = (BarrelStartLocation2D - AimJointLocation2D).Size();
	const float AimJoint_To_BarrelEnd_Distance = (BarrelEndLocation2D - AimJointLocation2D).Size();

	const FVector2D AimJoint_To_Target = ( TargetLocation2D - AimJointLocation2D );
	const float AimJoint_To_Target_Distance = AimJoint_To_Target.Size();

	float MinimumDistance = FMath::Min( AimJoint_To_BarrelStart_Distance, AimJoint_To_BarrelEnd_Distance );

	// Push the MinimumDistance outwards a little bit, just so that we're 100% sure we have a valid value.
	const float MinimumDistance_Tolerance = FMath::Min( 3.0f, 0.01f * MinimumDistance );
	MinimumDistance += MinimumDistance_Tolerance;

	if ( AimJoint_To_Target_Distance < MinimumDistance )
	{
		// The Target is at an invalid location, so return a location that is a little farther away.
		return AimJoint_To_Target.GetSafeNormal() * MinimumDistance;
	}

	// The Target is at a valid location, so just retun that.
	return TargetLocation2D;
}

bool UTurretRotationFunctionLibrary::CalculateBarrelRayDistance( 
	const FVector2D& AimJointLocation2D, 
	const FVector2D& BarrelStartLocation2D, 
	const FVector2D& BarrelEndLocation2D, 
	const FVector2D& TargetLocation2D, 
	float& Out_BarrelRayDistance )
{
	// Let:
	// J = AimJoint
	// S = BarrelStart
	// E = BarrelEnd
	// R = BarrelRay = (E - S).GetSafeNormal()
	// T = Target
	// d = BarrelRay distance we're trying to find (Out_BarrelRayDistance)
	// F(d)  = A point at distance "d" along the Barrel Ray
	//       = S + (R*d)
	// ||V|| = Calculates the magnitude (distance) of the 2D vector V.
	//       = V.Size()
	//       = sqrt(V.x^2 + V.y^2)

	// We're trying to find a value for "d" that solves this equation:
	// || F(d) - J || = || T - J ||

	// After a lot of algebra (omitted here), we can rewrite this as a quadratic equation:
	// a*(d^2) + b*d + c, where
	// a = (R.x^2) + (R.y^2)
	// b = (2*S.x*R.x - 2*J.x*R.x) + (2*S.y*R.y - 2*J.y*R.y)
	// c = (J.x-S.x)^2 + (J.y-S.y)^2 - (T.x-J.x)^2 - (T.y-J.y)^2

	// Using the qudratic formula, we can find calculate the roots, d1 and d2.
	// Then we select the "best" of these roots in SelectBestRayDistance.

	// Declare and calculate our quadratic coefficients.
	float a = 0.0f;
	float b = 0.0f;
	float c = 0.0f;
	CalculateQuadraticCoefficients( AimJointLocation2D, BarrelStartLocation2D, BarrelEndLocation2D, TargetLocation2D, /*out*/ a, /*out*/ b, /*out*/ c );

	// Using our quadratic coefficients, find out roots.
	float d1 = 0.0f;
	float d2 = 0.0f;
	bool bRootsWereFound = CalculateQuadraticRoots( a, b, c, /*out*/ d1, /*out*/ d2 );
	if ( !bRootsWereFound )
	{
		return false;
	}

	// Select the best root.
	Out_BarrelRayDistance = SelectBestRayDistance( d1, d2 );
	return true;
}

void UTurretRotationFunctionLibrary::CalculateQuadraticCoefficients( 
	const FVector2D& AimJointLocation2D, 
	const FVector2D& BarrelStartLocation2D, 
	const FVector2D& BarrelEndLocation2D, 
	const FVector2D& TargetLocation2D,
	float& Out_A, 
	float& Out_B, 
	float& Out_C )
{
	// Just for readability with CalculateBarrelRayDistance:
	const FVector2D& J = AimJointLocation2D;
	const FVector2D& S = BarrelStartLocation2D;
	const FVector2D& E = BarrelEndLocation2D;
	const FVector2D R = (E - S).GetSafeNormal();
	const FVector2D& T = TargetLocation2D;

	const float A_Term1 = FMath::Pow(R.X, 2);
	const float A_Term2 = FMath::Pow(R.Y, 2);

	const float B_Term1 = (2*S.X*R.X) - (2*J.X*R.X);
	const float B_Term2 = (2*S.Y*R.Y) - (2*J.Y*R.Y);

	const float C_Term1 = FMath::Pow(J.X-S.X, 2);
	const float C_Term2 = FMath::Pow(J.Y-S.Y, 2);
	const float C_Term3 = -FMath::Pow(T.X-J.X, 2);
	const float C_Term4 = -FMath::Pow(T.Y-J.Y, 2);

	Out_A = A_Term1 + A_Term2;
	Out_B = B_Term1 + B_Term2;
	Out_C = C_Term1 + C_Term2 + C_Term3 + C_Term4;
}

bool UTurretRotationFunctionLibrary::CalculateQuadraticRoots( float A, float B, float C, float& Out_X1, float& Out_X2 )
{
	// See https://en.wikipedia.org/wiki/Quadratic_equation

	const float Denominator = 2*A;
	if ( FMath::IsNearlyZero(Denominator) )
	{
		return false;
	}

	const float RadicalInput = FMath::Pow(B, 2) - (4*A*C);
	if ( RadicalInput < 0 )
	{
		return false;
	}

	const float Radical = FMath::Sqrt( RadicalInput );

	Out_X1 = ((-B) - (Radical)) / Denominator;
	Out_X2 = ((-B) + (Radical)) / Denominator;

	return true;
}

float UTurretRotationFunctionLibrary::SelectBestRayDistance( float FirstDistance, float SecondDistance )
{
	if ( FirstDistance < 0 && SecondDistance < 0 )
	{
		// I'm not sure if this case ever occurs, but just in case.
		// Both distances are "behind" the BarrelStart, which is odd, so just pick whichever one is "less behind" the BarrelStart.
		return FMath::Min( FirstDistance, SecondDistance );
	}

	// Else just return the largest value.  This may just select between a positive/negative value.
	// A positive value is "in front of" BarrelStart, which is what we want.
	return FMath::Max( FirstDistance, SecondDistance );
}

float UTurretRotationFunctionLibrary::CalculateAngleToRotateFromFirstVectorToSecondVector( FVector2D FirstVector, FVector2D SecondVector )
{
	const FVector2D FirstVector_Normalized = FirstVector.GetSafeNormal();
	const FVector2D SecondVector_Normalized = SecondVector.GetSafeNormal();

	// Find the angle between the two vectors.
	const float AngleBetweenAimJointVectors_Radians = CalculateAngleBetweenNormalizedVectors( FirstVector_Normalized, SecondVector_Normalized );
	const float AngleBetweenAimJointVectors_Degrees = FMath::RadiansToDegrees( AngleBetweenAimJointVectors_Radians );

	// If it's shorter to turn counterclockwise, then do so.  Else, let's turn clockwise.
	const float RotationSign = ShouldTurnCounterClockwiseToMeet( FirstVector_Normalized, SecondVector_Normalized ) ? 1.0f : -1.0f;

	return RotationSign * AngleBetweenAimJointVectors_Degrees;
}

float UTurretRotationFunctionLibrary::CalculateAngleBetweenNormalizedVectors( FVector2D FirstVector_Normalized, FVector2D SecondVector_Normalized )
{
	// The DotProduct is defined as:  Length(A) * Length(B) * (cosine(Angle between A and B))
	// See https://en.wikipedia.org/wiki/Dot_product#Geometric_definition

	// So, since we know these two vectors are normalized, we can assume that the DotProduct = cosine(Angle between A and B)
	const float DotProduct = FirstVector_Normalized | SecondVector_Normalized;
	return FMath::Acos( DotProduct );
}

bool UTurretRotationFunctionLibrary::ShouldTurnCounterClockwiseToMeet( FVector2D FirstVector, FVector2D SecondVector )
{
	// If the SecondVector "is in front of" FirstVector_Perp (positive dot product), then we should rotate counterclockwise.
	const FVector2D FirstVector_Perp = FirstVector.GetRotated( 90 );

	return ( FirstVector_Perp | SecondVector ) >= 0;
}

