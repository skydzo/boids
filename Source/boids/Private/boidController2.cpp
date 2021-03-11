// Fill out your copyright notice in the Description page of Project Settings.


#include "boidController2.h"

AboidController2::AboidController2()
{
	PrimaryActorTick.bCanEverTick = true;
	PathSpline = CreateDefaultSubobject<USplineComponent>(TEXT("Path"));


}

void AboidController2::BeginPlay()
{
	Super::BeginPlay();

	BoidsMeshInstance = NewObject<UInstancedStaticMeshComponent>(this);
	BoidsMeshInstance->RegisterComponent();
	BoidsMeshInstance->SetStaticMesh(boidsMesh);
	BoidsMeshInstance->SetGenerateOverlapEvents(false);
	BoidsMeshInstance->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	BoidsMeshInstance->CastShadow = false;

	PredatorsMeshInstance = NewObject<UInstancedStaticMeshComponent>(this);
	PredatorsMeshInstance->RegisterComponent();
	PredatorsMeshInstance->SetStaticMesh(predatorMesh);
	PredatorsMeshInstance->SetGenerateOverlapEvents(false);
	PredatorsMeshInstance->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	PredatorsMeshInstance->CastShadow = false;

	centerPositon = GetActorLocation();
	FVector4 pos;
	FRotator rot;
	TArray<FVector4> emptyArray;


	int index = 0;
	for (int i = 0; i < boidTypeNumber; i++) {
		TArray<FVector4> boidsPos;
		for (int j = 0; j < (boidNumber / boidTypeNumber); j++) {
			pos = FVector4(((float(FMath::RandRange(0, 1000)) / 1000.0f) * boidsRange) + centerPositon.X, ((float(FMath::RandRange(0, 1000)) / 1000.0f) * boidsRange) + centerPositon.Y, ((float(FMath::RandRange(0, 1000)) / 1000.0f) * boidsRange) + centerPositon.Z,index);
			rot = FRotator(float(FMath::RandRange(-180, 180)), float(FMath::RandRange(-180, 180)), float(FMath::RandRange(-180, 180)));
			BoidsMeshInstance->AddInstanceWorldSpace(FTransform(rot, pos, FVector(boidScale, boidScale, boidScale)));
			boidsPos.Push(pos);
			newBoidsTransformsArray.Push(FTransform(rot,pos, FVector(boidScale, boidScale, boidScale)));
			neighbors.Push(emptyArray);
			index++;
		}
		boids.Add(i, boidsPos);
	}

	for (int i = 0; i < nbPredator; i++) {
		pos = FVector4(((float(FMath::RandRange(0, 1000)) / 1000.0f) * boidsRange) + centerPositon.X, ((float(FMath::RandRange(0, 1000)) / 1000.0f) * boidsRange) + centerPositon.Y, ((float(FMath::RandRange(0, 1000)) / 1000.0f) * boidsRange) + centerPositon.Z,i);
		rot = FRotator(float(FMath::RandRange(-180, 180)), float(FMath::RandRange(-180, 180)), float(FMath::RandRange(-180, 180)));
		PredatorsMeshInstance->AddInstanceWorldSpace(FTransform(rot,pos,FVector(predatorScale, predatorScale, predatorScale)));
		predators.Push(pos);
	}
}

void AboidController2::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	FTransform tmp;

	//newBoidsTransformsArray.Empty();
	newPredatorsTransformsArray.Empty();

	if (!activateBoids) {
		for (int i = 0; i < boidTypeNumber; i++) {
			for (int j = 0; j < boids.Find(i)->Num(); j++) {
				BoidsMeshInstance->GetInstanceTransform(boids[i][j].W, tmp);
				FVector newDirection = tmp.GetRotation().GetForwardVector();
				keepInBounds(newDirection, tmp.GetLocation());
				limitSpeed(newDirection);
				newBoidsTransformsArray.Push(FTransform(calcNewDirection(newDirection, tmp.GetLocation()), tmp.GetLocation() + newDirection, tmp.GetScale3D()));
				boids[i][j] = FVector4(tmp.GetLocation() + newDirection, boids[i][j].W);
			}
		}
	}
	else {
		currentMaxNeigbors = 0;
		if (!bIsfollowingSpline) {
			for (int i = 0; i < boidTypeNumber; i++) {
				ParallelFor(boids.Find(i)->Num(), [&](int32 j)
				{
					getNeighbors(j, i, boids[i][j].W, boids[i][j]);
				});

				for (int j = 0; j < boids.Find(i)->Num(); j++) {
					BoidsMeshInstance->GetInstanceTransform(boids[i][j].W, tmp);
					FVector newDirection = tmp.GetRotation().GetForwardVector();
					//getNeighbors(j, i, boids[i][j].W, boids[i][j]);
					cohesion(boids[i][j].W, newDirection, boids[i][j]);
					averageAlignment(boids[i][j].W, newDirection, tmp.GetRotation().GetForwardVector());
					separate(boids[i][j].W, newDirection, boids[i][j]);
					moveFromPredators(newDirection, boids[i][j]);
					moveFromColision(newDirection, boids[i][j]);
					keepInBounds(newDirection, tmp.GetLocation());
					limitSpeed(newDirection);
					newBoidsTransformsArray[boids[i][j].W] = FTransform(calcNewDirection(newDirection, tmp.GetLocation()), tmp.GetLocation() + newDirection, tmp.GetScale3D());
					boids[i][j] = FVector4(tmp.GetLocation() + newDirection, boids[i][j].W);
				}
			}
		}
		else {	
			for (int i = 0; i < boidTypeNumber; i++) {
				ParallelFor(boids.Find(i)->Num(), [&](int32 j)
				{
					getNeighbors(j, i, boids[i][j].W, boids[i][j]);
				});

				for (int j = 0; j < boids.Find(i)->Num(); j++) {
					BoidsMeshInstance->GetInstanceTransform(boids[i][j].W, tmp);
					FVector newDirection = tmp.GetRotation().GetForwardVector();
					cohesion(boids[i][j].W, newDirection, boids[i][j]);
					averageAlignment(boids[i][j].W, newDirection, tmp.GetRotation().GetForwardVector());
					separate(boids[i][j].W, newDirection, boids[i][j]);
					moveFromPredators(newDirection, boids[i][j]);
					followSpline(newDirection, boids[i][j], boids[i][j].W ,FMath::Lerp(0.0f, PathSpline->GetSplineLength(), float(alpha)/ float(alphaMod)));
					limitSpeed(newDirection);
					newBoidsTransformsArray[boids[i][j].W] = FTransform(calcNewDirection(newDirection, tmp.GetLocation()), tmp.GetLocation() + newDirection, tmp.GetScale3D());
					boids[i][j] = FVector4(tmp.GetLocation() + newDirection, boids[i][j].W);
				}
			}
			alpha++;
			alpha = alpha%alphaMod;
		}



		for (int i = 0; i < predators.Num(); i++) {
			PredatorsMeshInstance->GetInstanceTransform(predators[i].W, tmp);
			FVector newDirection = tmp.GetRotation().GetForwardVector();
			if (centeredBoid != FVector(0, 0, 0)) {
				huntBoids(newDirection, predators[i]);
			}
			separatePredators(newDirection, predators[i]);
			limitPredatorSpeed(newDirection);
			keepInBounds(newDirection, predators[i]);
			calcNewDirection(newDirection, predators[i]);
			newPredatorsTransformsArray.Push(FTransform(calcNewDirection(newDirection, tmp.GetLocation()), tmp.GetLocation() + newDirection, tmp.GetScale3D()));
			predators[i] = FVector4(tmp.GetLocation() + newDirection, predators[i].W);
		}
	}

	BoidsMeshInstance->BatchUpdateInstancesTransforms(0, newBoidsTransformsArray, true, true, false);
	PredatorsMeshInstance->BatchUpdateInstancesTransforms(0, newPredatorsTransformsArray, true, true, false);

}

void AboidController2::keepInBounds(FVector &direction, FVector4 position) {
	if (position.X < centerPositon.X - boidsRange) {
		direction.X = direction.X + 0.5f;
	}

	if (position.X > centerPositon.X + boidsRange) {
		direction.X = direction.X - 0.5f;
	}

	if (position.Y < centerPositon.Y - boidsRange) {
		direction.Y = direction.Y + 0.5f;
	}

	if (position.Y > centerPositon.Y + boidsRange) {
		direction.Y = direction.Y - 0.5f;
	}

	if (position.Z < centerPositon.Z - boidsRange) {
		direction.Z = direction.Z + 0.5f;
	}

	if (position.Z > centerPositon.Z + boidsRange) {
		direction.Z = direction.Z - 0.5f;
	}

}

void AboidController2::separate(int boidIndex, FVector &direction, FVector position) {
	FVector moveAway = FVector(0, 0, 0);

	for (int i = 0; i < neighbors[boidIndex].Num(); i++) {
		if (FVector::Dist(position, neighbors[boidIndex][i]) < separation) {
			moveAway.X += position.X - neighbors[boidIndex][i].X;
			moveAway.Y += position.Y - neighbors[boidIndex][i].Y;
			moveAway.Z += position.Z - neighbors[boidIndex][i].Z;
		}
	}
	direction.X += moveAway.X * separationFact;
	direction.Y += moveAway.Y * separationFact;
	direction.Z += moveAway.Z * separationFact;
}

void AboidController2::separatePredators(FVector &direction, FVector location) {
	FVector moveAway = FVector(0, 0, 0);
	for (int i = 0; i < predators.Num(); i++) {
		if (FVector::Dist(location, predators[i]) < predatorSeparation) {
			moveAway.X += location.X - predators[i].X;
			moveAway.Y += location.Y - predators[i].Y;
			moveAway.Z += location.Z - predators[i].Z;
		}
	}
	direction.X += moveAway.X * predatorSeparationFact;
	direction.Y += moveAway.Y * predatorSeparationFact;
	direction.Z += moveAway.Z * predatorSeparationFact;
}


void AboidController2::averageAlignment(int boidIndex, FVector &direction,FVector startDirection) {
	FVector avgDirection = FVector(0, 0, 0);
	FTransform neighborsTransform;
	FVector forwardVector;
	FVector currentDir = startDirection;


	for (int i = 0; i < neighbors[boidIndex].Num(); i++) {
		BoidsMeshInstance->GetInstanceTransform(neighbors[boidIndex][i].W, neighborsTransform);
		forwardVector = neighborsTransform.GetRotation().GetForwardVector();
		avgDirection.X = avgDirection.X + forwardVector.X;
		avgDirection.Y = avgDirection.Y + forwardVector.Y;
		avgDirection.Z = avgDirection.Z + forwardVector.Z;
	}

	if (neighbors[boidIndex].Num() > 0) {
		FTransform tmp;
		avgDirection = FVector(avgDirection.X / neighbors[boidIndex].Num(), avgDirection.Y / neighbors[boidIndex].Num(), avgDirection.Z / neighbors[boidIndex].Num());
		direction.X = direction.X + ((avgDirection.X - startDirection.X)* alignmentFact);
		direction.Y = direction.Y + ((avgDirection.Y - startDirection.Y)* alignmentFact);
		direction.Z = direction.Z + ((avgDirection.Z - startDirection.Z)* alignmentFact);
	}
}

void AboidController2::cohesion(int boidIndex, FVector &direction, FVector location) {
	FVector centerMass = FVector(0, 0, 0);
	for (int i = 0; i < neighbors[boidIndex].Num(); i++) {
		centerMass.X = centerMass.X + neighbors[boidIndex][i].X;
		centerMass.Y = centerMass.Y + neighbors[boidIndex][i].Y;
		centerMass.Z = centerMass.Z + neighbors[boidIndex][i].Z;
	}

	if (neighbors[boidIndex].Num() > 0) {
		centerMass = FVector(centerMass.X / neighbors[boidIndex].Num(), centerMass.Y / neighbors[boidIndex].Num(), centerMass.Z / neighbors[boidIndex].Num());
		direction.X = direction.X + ((centerMass.X - location.X)*coherenceFact);
		direction.Y = direction.Y + ((centerMass.Y - location.Y)*coherenceFact);
		direction.Z = direction.Z + ((centerMass.Z - location.Z)*coherenceFact);
	}
}

void AboidController2::moveFromPredators(FVector &direction, FVector location) {
	for (int i = 0; i < predators.Num(); i++) {
		if (FVector::Dist(location, predators[i]) < predatorVisualRange) {
			direction.X += (location.X - predators[i].X) * predatorDodgeFact;
			direction.Y += (location.Y - predators[i].Y) * predatorDodgeFact;
			direction.Z += (location.Z - predators[i].Z) * predatorDodgeFact;
		}
	}
}

void AboidController2::moveFromColision(FVector &direction, FVector location) {

}

void AboidController2::huntBoids(FVector &direction, FVector location) {
	direction.X += (centeredBoid.X - location.X) * predatorHuntFact;
	direction.Y += (centeredBoid.Y - location.Y) * predatorHuntFact;
	direction.Z += (centeredBoid.Z - location.Z) * predatorHuntFact;
}

void AboidController2::limitSpeed(FVector &direction) {
	float speed = FMath::Sqrt(direction.X*direction.X + direction.Y*direction.Y + direction.Z*direction.Z);
	if (speed > speedLimit) {
		direction.X = (direction.X / speed) * speedLimit;
		direction.Y = (direction.Y / speed) * speedLimit;
		direction.Z = (direction.Z / speed) * speedLimit;
	}
}

void AboidController2::limitPredatorSpeed(FVector &direction) {
	float speed = FMath::Sqrt(direction.X*direction.X + direction.Y*direction.Y + direction.Z*direction.Z);
	if (speed > predatorSpeedLimit) {
		direction.X = (direction.X / speed) * predatorSpeedLimit;
		direction.Y = (direction.Y / speed) * predatorSpeedLimit;
		direction.Z = (direction.Z / speed) * predatorSpeedLimit;
	}
}

void AboidController2::followSpline(FVector &direction, FVector location, int globalID, float splinePoint) {
	splinePoint = (int32)(splinePoint  + (5.0f * float(globalID)))% (int32)PathSpline->GetSplineLength();
	FVector target = PathSpline->GetLocationAtDistanceAlongSpline(splinePoint,ESplineCoordinateSpace::World);
	direction.X += (target.X - location.X) * splineFact;
	direction.Y += (target.Y - location.Y) * splineFact;
	direction.Z += (target.Z - location.Z) * splineFact;
}


FRotator AboidController2::calcNewDirection(FVector direction,FVector loc) {
	return UKismetMathLibrary::FindLookAtRotation(loc, loc + (direction * 10.0f));
}

void AboidController2::getNeighbors(int boidIndex, int boidFamily,int boidGlobalIndex,FVector location) {
	neighbors[boidGlobalIndex].Empty();
	for (int i = 0; i < boids.Find(boidFamily)->Num(); i++) {
		if (i != boidIndex) {
			if (FVector::Dist(boids[boidFamily][i], location) < visualRange) {
				neighbors[boidGlobalIndex].Push(boids[boidFamily][i]);
			}
		}
	}

	if (neighbors[boidGlobalIndex].Num() > currentMaxNeigbors) {
		currentMaxNeigbors = neighbors[boidGlobalIndex].Num();
		centeredBoid = boids[boidFamily][boidIndex];
	}
}

