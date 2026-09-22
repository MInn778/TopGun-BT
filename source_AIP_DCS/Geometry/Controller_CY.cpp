#include "Controller_CY.h"
#include <math.h>

float clamp(float input, float RangeDown, float RangeUp)
{
	if (input <= RangeDown)
	{
		return RangeDown;
	}
	else if (input >= RangeUp)
	{
		return RangeUp;
	}
	else
	{
		return input;
	}
}

StickController::StickController()
{
	SumCount = 0;
	for (int i = 0; i < 20; i++)
		MF[i] = 0;
	FilterIndex = 0;

	for (int i = 0; i < 60; i++)
		ErrorSum.push_back(0.0);

	m_hasPrevRollCMD = false;
	m_prevRollCMD = 0.0f;
}

float StickController::GetLOSErrorSUM(float LOSError)
{
	SumCount++;
	if (LOSError <= 10)
	{
		if (SumCount < 60)
			ErrorSum.push_back(LOSError);
		else
			ErrorSum[SumCount % 60] = LOSError;
	}
	else
	{
		if (ErrorSum.size() < 60)
			ErrorSum.push_back(0);
		else
			ErrorSum[SumCount % 60] = 0;
	}
	int sum = 0;

	for (int i = 0; i < ErrorSum.size(); i++)
	{
		sum += ErrorSum[i];
	}

	float Re = sum / 60;

	return Re;
}

StickValue StickController::GetStick(Vector3 MyLocation_FNED, Vector3 MyRotation_FNED, Vector3 VP)
{
	Vector3 Mylocation(MyLocation_FNED.X, MyLocation_FNED.Y, MyLocation_FNED.Z);
	Vector3 TargetLocation(VP.X, VP.Y, VP.Z);

	//오일러 각을 입력. 이 부분은 언리얼4의 각도를 회사의 ECEF_LLA_Converter 쪽의 각도와 함수들을 이용하기 위해 이쪽 양식에 맞추는 과정
	EulerAngle EA;
	EA.Roll = MyRotation_FNED.X;
	EA.Pitch = MyRotation_FNED.Y;
	EA.Yaw = MyRotation_FNED.Z;

	//오일러각을 이용하면 축변화에 따른 오차가 생기기 때문에 쿼터니언으로 변환하여 사용
	Quaternion QU = EA.toQuaternion();

	//쿼터니언을 이용하여 전방벡터(ForwardVector)를 생성 
	Vector3 ForwardVector;
	ForwardVector.X = 1 - 2 * (QU.X * QU.X + QU.Y * QU.Y);
	ForwardVector.Y = 2 * (QU.X * QU.Z + QU.W * QU.Y);
	ForwardVector.Z = -2 * (QU.Y * QU.Z - QU.W * QU.X);

	//쿼터니언을 이용하여 수직벡터(UpVector)를 생성 
	Vector3 UpVector;
	UpVector.X = -2 * (QU.Y * QU.Z + QU.W * QU.X);
	UpVector.Y = -2 * (QU.X * QU.Y - QU.W * QU.Z);
	UpVector.Z = 1 - 2 * (QU.X * QU.X + QU.Z * QU.Z);

	//쿼터니언을 이용하여 오른쪽벡터(RightVector)를 생성 
	Vector3 RightVector;
	RightVector.X = 2 * (QU.X * QU.Z - QU.W * QU.Y);
	RightVector.Y = 1 - 2 * (QU.Y * QU.Y + QU.Z * QU.Z);
	RightVector.Z = -2 * (QU.X * QU.Y + QU.W * QU.Z);


	Vector3 ForwardVectorPoint = ForwardVector * 1000 + Mylocation;

	Vector3 ForwardVectorPoint2VP = TargetLocation - ForwardVectorPoint;

	Vector3 Proj_V = (ForwardVectorPoint2VP.dot(ForwardVector)) * ForwardVector;

	Vector3 Proj_P = TargetLocation - Proj_V;
	Vector3 Proj_TV = Proj_P - ForwardVectorPoint;

	// 롤커멘드 생성 부분

	float UpVector2Proj_TV_Angle = std::acos(UpVector.dot(Proj_TV / Proj_TV.length()));
	float UTAngle;
	float LOS = std::acos(ForwardVector.dot((TargetLocation - Mylocation)) / (TargetLocation - Mylocation).length()) * RADTODEG;

	if (_isnan(UpVector2Proj_TV_Angle) != 0)
	{
		UpVector2Proj_TV_Angle = 0;
	}

	float Proj_TV_Length = Proj_TV.length();

	if(Proj_TV_Length <= 0)
	{
		Proj_TV_Length = 0.0001;
	}

	if (RightVector.dot(Proj_TV / Proj_TV_Length) >= 0)
	{
		UTAngle = UpVector2Proj_TV_Angle;
	}
	else
	{
		UTAngle = UpVector2Proj_TV_Angle * (-1);
	}

	float RollCMD;

	if (std::abs(UTAngle * RADTODEG) > 90)
	{
		RollCMD = (std::sin(UTAngle) * 1);

		if (LOS > 3)
			RollCMD = clamp(RollCMD, -1, 1);
		else
			RollCMD = RollCMD * LOS * (-0.1);
	}
	else
	{
		RollCMD = (std::sin(UTAngle) * 1.0);

		RollCMD = clamp(RollCMD, -1, 1);

		RollCMD = RollCMD * std::abs(RollCMD);
	}


	if (_isnan(LOS) != 0)
	{
		LOS = 0;
	}

	if (RollCMD < 0.1)
		RollCMD = RollCMD * 3;

	RollCMD = RollCMD * clamp(LOS, 0, 1);

	// 2026-08-17 9번째 시도: 이전 8번은 전부 피치(상승 명령) 쪽을 건드렸으나
	// 전부 효과 없었음(자멸률 그대로/악화). 스로틀은 모든 Task에서 이미 항상
	// 1.0으로 고정돼 있어 "추력 부족"은 원인이 아님 -- 대신 급격한 뱅크(횡전)로
	// 인한 유도항력이 저고도 추격/교전 중 에너지(속도)를 갉아먹어, 막상
	// EmergencyPullUp이 발동해도 이미 상승할 에너지가 없는 상태로 진입하는
	// 것으로 추정. 저고도일수록 롤 커맨드 크기 자체를 제한(수평 비행 우선,
	// 에너지 보존)해서 급뱅크로 인한 에너지 소모를 줄여본다.
	// 2026-08-17 미세조정 시도: 트리거를 2800m/제한을 0.3까지 세게 조여봤으나
	// 승53.3%/패40.0%로 2000m/0.4 버전(승53.3%/패38.3%)보다 소폭 나빠짐(노이즈
	// 범위지만 개선 아님) -- 2000m/0.4로 확정.
	{
		double altitudeForRollCap = Mylocation.Z;
		if (altitudeForRollCap < 2000.0)
		{
			double urgency = (2000.0 - altitudeForRollCap) / 2000.0; // 0~1
			if (urgency > 1.0) urgency = 1.0;
			float rollCap = (float)(1.0 - 0.6 * urgency); // 2000m: 1.0(무제한), 0m: 0.4
			RollCMD = clamp(RollCMD, -rollCap, rollCap);
		}
	}
	//러더 커맨드 생성 부분
	float RudderCMD = 0;

	RudderCMD = -std::sin(UTAngle) * clamp(LOS, 0, 6) * 1;

	MF[FilterIndex % 20] = RudderCMD;
	FilterIndex++;

	int MFsum = 0;
	for (int i = 0; i < 20; i++)
		MFsum += MF[i];
	RudderCMD = (MFsum / 20 + RudderCMD) / 2;

	//피치 커맨드 생성 부분
	float PitchCMD = 0;;

	float ERROR_Effect = clamp(LOS / 6 + clamp(GetLOSErrorSUM(LOS) / 7.5, 0, 0.25), 0, 1.5);
	//float ERROR_Effect = clamp(LOS / 6, 0, 1.5);


	float Roll_Effect = 1 - clamp(std::abs(UTAngle * RADTODEG) / 90, 0, 1);

	float Horizon_Effect;
	if (std::abs(UTAngle * RADTODEG) <= 90)
	{
		Horizon_Effect = 1;
	}
	else
		Horizon_Effect = 0.5;

	//std::cout << "ERROR_Effect : " << ERROR_Effect << " Roll_Effect : " << Roll_Effect << " Horizon_Effect : " << Horizon_Effect << std::endl;

	if (LOS < 90)
		PitchCMD = ERROR_Effect * Roll_Effect * Horizon_Effect * (-1);//+Roll_Effect2;
	else
		PitchCMD = -1;

	// 2026-08-17: 컨트롤러 레벨 최소 상승 바닥값(PitchCMD floor)을 뱅크각
	// 제한 없이 단독으로 검증했을 땐 승45.0%/패48.3%로 악화(원래 버전으로
	// 되돌렸었음). 그때는 급뱅크로 이미 에너지가 바닥난 상태에서 억지로 더
	// 당기려 한 게 원인으로 추정했고, 위(RollCMD)의 저고도 뱅크각 제한과
	// 조합(+완화된 바닥값 -0.35->-0.2, 트리거 3000->2000m)해서 60개 배치로
	// 재검증한 결과 승53.3%->55.0%, 패38.3%->36.7%로 실제 개선 확인 -- 최종 채택.
	{
		double altitudeForPitchFloor = Mylocation.Z;
		if (altitudeForPitchFloor < 2000.0)
		{
			double urgency = (2000.0 - altitudeForPitchFloor) / 2000.0; // 0~1
			if (urgency > 1.0) urgency = 1.0;
			double climbFloor = -0.2 * urgency; // 2000m: 0, 0m: -0.2
			if (PitchCMD > climbFloor)
			{
				PitchCMD = (float)climbFloor;
			}
		}
	}

	// 2026-08-22(v2에서 포팅, 2차 시도 -- 0.05->0.12): 1차(0.05)는 라이브에서
	// 방향전환이 눈에 띄게 느려짐(실측: 거리 좁혀지는 중에도 롤이 47.7->42.8도로
	// 4초 가까이 거의 정체) -- v2 자신의 틱 주기 기준으로 캡을 잡아서 v1에
	// 그대로 옮기면 굼떠지는 것으로 추정. 요동 억제(원래 목적)와 반응성
	// 사이의 중간값으로 재시도.
	RollCMD = clamp(RollCMD, -1, 1);
	if (m_hasPrevRollCMD)
	{
		float maxDelta = 0.12f;
		float delta = RollCMD - m_prevRollCMD;
		if (delta > maxDelta) delta = maxDelta;
		if (delta < -maxDelta) delta = -maxDelta;
		RollCMD = m_prevRollCMD + delta;
	}
	m_prevRollCMD = RollCMD;
	m_hasPrevRollCMD = true;

	StickValue Result;
	Result.RollCMD = clamp(RollCMD, -1, 1);
	Result.PitchCMD = clamp(PitchCMD, -1, 1);
	Result.RudderCMD = clamp(RudderCMD, -1, 1);
	//Result.RudderCMD = RudderCMD;
	return Result;
}
