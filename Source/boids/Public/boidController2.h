// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"
#include "Math/UnrealMathUtility.h"
#include "Kismet/KismetMathLibrary.h"
#include "Components/SplineComponent.h"
#include "boidController2.generated.h"

UCLASS()
class BOIDS_API AboidController2 : public AActor
{
	GENERATED_BODY()

public:	
	// Sets default values for this actor's properties
	AboidController2();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		class UStaticMesh* boidsMesh;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		class UStaticMesh* predatorMesh;
	UPROPERTY(EditAnywhere, Instanced, Category = "Path spline")
		USplineComponent* PathSpline;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		int nbPredator = 1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		float predatorScale = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		float predatorVisualRange = 30;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		float predatorSpeedLimit = 1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		float predatorSeparation = 8;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		float predatorSeparationFact = 0.05f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		float predatorHuntFact = 0.005f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		float predatorDodgeFact = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Colision")
		float collisionDodgeFact = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float boidsRange = 45;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		int boidNumber = 220;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		int boidTypeNumber = 1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float boidScale = 0.5f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float speedLimit = 0.5f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float turnFactor = 1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float coherenceFact = 0.005f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float alignmentFact = 0.05f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float separation = 8;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float separationFact = 0.05f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float splineFact = 0.005f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float visualRange = 16;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		bool bIsfollowingSpline = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		bool activateBoids = false;

	TArray<FTransform> newBoidsTransformsArray;
	TArray<FTransform> newPredatorsTransformsArray;
	TArray<FTransform> predBoidsTransformsArray;
	TArray<FTransform> predPredatorsTransformsArray;
	UInstancedStaticMeshComponent *BoidsMeshInstance;
	UInstancedStaticMeshComponent *PredatorsMeshInstance;

	FVector centerPositon;

	int currentMaxNeigbors = 0;
	FVector centeredBoid;

	TMap<int, TArray<FVector4>> boids;
	TArray<TArray<FVector4>> neighbors;
	TArray<FVector4> predators;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
	int alpha = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
	int alphaMod = 10000;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	void getNeighbors(int boidIndex, int boidFamily, int boidGlobalIndex, FVector location);

	void keepInBounds(FVector &direction, FVector4 position);

	void separate(int boidIndex, FVector &direction,FVector position);

	void separatePredators(FVector &direction, FVector location);

	void averageAlignment(int boidIndex, FVector &direction, FVector startDirection);

	void cohesion(int boidIndex, FVector &direction, FVector location);

	void moveFromPredators(FVector &direction, FVector location);

	void moveFromColision(FVector &direction, FVector location);

	void huntBoids(FVector &direction, FVector location);

	void limitSpeed(FVector &direction);

	void limitPredatorSpeed(FVector &direction);

	void followSpline(FVector &direction, FVector location, int globalID, float alpha);

	FRotator calcNewDirection(FVector direction,FVector loc);

	/*UFUNCTION(Blueprintable, BlueprintCallable, Category = "data")
		TArray<int> getBoidsByFamily(int familyID);*/

};
