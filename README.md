# ZED 2i Anchor Hole Inspection

Stereolabs **ZED 2i**의 왼쪽 RGB 영상과 스테레오 XYZ 포인트클라우드를 결합해 평면 작업면의 앙카홀 입구를 찾는 C# WPF 프로그램입니다.

카메라 연동은 Stereolabs 공식 [`zed-csharp-api`](https://github.com/stereolabs/zed-csharp-api)의 NuGet 배포본 `Stereolabs.zed 5.4.0`을 사용합니다. 참고 프로젝트인 [`Helios2_Ray_AnchorHole_Inspection`](https://github.com/kimAssembly/Helios2_Ray_AnchorHole_Inspection)의 UI·ROI·시간축 안정화 개념은 참고했지만, ToF 깊이 피크 검출과 Arena SDK 코드는 사용하지 않습니다.

## 핵심 알고리즘

ZED 2i는 능동 ToF 카메라가 아니라 수동 스테레오 카메라이므로 홀 내부에서 깊이값이 사라질 수 있습니다. 이 프로젝트는 다음 순서로 처리합니다.

1. 왼쪽 RGB 영상을 회색조로 변환합니다.
2. 다중 반지름 스캔으로 중심부가 어둡고 바깥쪽이 밝은 원형·방사형 후보를 찾습니다.
3. 후보끼리 겹치면 가장 강한 후보만 남깁니다.
4. 각 후보의 **홀 바깥 링**에서만 유효 XYZ를 수집합니다.
5. 링 포인트에 RANSAC과 공분산 재적합으로 국소 평면을 계산합니다.
6. 평면 포인트가 후보 주위를 충분한 각도로 둘러싸는지 확인합니다.
7. 홀 내부에 평면보다 깊은 XYZ가 있거나, 원형 내부에 스테레오 무효점이 집중되면 홀로 판정합니다.
8. 홀 내부가 무효여도 링의 픽셀↔3D 국소 사상을 이용해 **표면상의 홀 입구 중심 XYZ**와 지름을 계산합니다.
9. 3프레임 연속 확인된 결과만 표시하고 시간축으로 평활화합니다.

## 요구 환경

- Windows 10/11 x64
- .NET 8 SDK 이상
- NVIDIA CUDA 지원 GPU
- ZED SDK 5.4.x
- ZED 2i USB 3.x 연결

현재 개발 PC에서 확인된 환경은 ZED SDK `5.4.1`, ZED 2i USB 장치 `VID_2B03&PID_F880`, NVIDIA RTX 4060 Laptop GPU입니다.

실기 재연결 후 공식 C# API에서 ZED SDK 장치 1대, ZED 2i S/N `31045332`, RGB `1280×720`, XYZ `640×360` 프레임 취득까지 통과했습니다. 첫 시험 프레임의 정중앙 XYZ 한 점은 무효였으므로, 실제 검출 시험에서는 질감이 있는 평면을 약 0.5–2 m 거리에서 촬영하고 ZED Depth Viewer로 주변 유효 깊이 분포를 함께 확인해야 합니다.

실기 화면에서 `RGB candidates > 0`, `XYZ validated = 0`이 확인되어 다음을 보완했습니다.

- 홀 경계를 보존하도록 ZED 깊이 모드를 `ULTRA`로 변경
- 상위 24개로 잘리던 RGB 후보를 전부 3D 검증
- 250 mm보다 먼 배경이 보이는 관통홀을 `DistantBackground` 증거로 허용
- 상태 표시줄에 검사 후보, 유효 림, 평면, 홀 증거, ROI의 XYZ 유효률 표시

검출되지 않을 때 상태 표시줄을 먼저 확인합니다. `림 0`이면 홀 주변의 유효 깊이가 부족한 것이고, `평면 0`이면 주변 표면이 흔들리거나 평면 허용 오차를 벗어난 것입니다. `평면 > 0`, `홀증거 0`이면 영상에는 검은 원이 있지만 XYZ에는 함몰·무효점·먼 배경이 없어 인쇄 무늬로 판정된 것입니다.

### 정밀 모드와 오탐 제한

실기에서 약 207 mm 거리, ROI XYZ 유효률 26%인 장면의 인쇄 문자와 테이프 경계가 `StereoVoid`로 오인되는 현상을 확인했습니다. 정밀 모드는 다음 조건을 추가로 적용합니다.

- 대상 표면 거리 기본 300 mm 이상(권장 운용 거리 0.4–1.5 m)
- 실제 홀 지름 기본 8–60 mm
- 후보 내부가 원형으로 어둡게 채워진 비율 52% 이상
- 후보 둘레 XYZ 유효률 65% 이상
- 실제로 더 깊은 XYZ 또는 관통홀 뒤의 먼 유효 XYZ가 존재
- 5회 연속 검출 후에만 화면에 안정 검출로 표시

v3에서는 실제 장면에서 모든 후보가 거리/지름 단계에서 탈락한 결과를 반영해 기본 지름을 5–80 mm, 최소 거리를 250 mm로 조정했습니다. 무효 깊이는 16방향의 원 경계 반지름 일관성, 어두운 내부 채움률, 둘레 XYZ 유효률, 내부와 둘레의 무효점 비율 차이를 모두 통과할 때만 제한적으로 허용합니다. 상태 표시줄에는 `거리`와 `지름` 통과 개수가 각각 표시됩니다.

스테레오 깊이가 없는 검은 원형 인쇄와 깊이가 없는 실제 구멍은 카메라 정보만으로 완전히 구별할 수 없습니다. 현장 홀의 실제 지름 범위를 좁게 지정하는 것이 가장 효과적인 추가 오탐 제한입니다.

### 100 mm 근접 운용

ZED 2i 2.1 mm 모델의 공식 깊이 범위는 약 0.3–20 m이므로 100 mm에서는 RGB 영상과 신뢰 가능한 XYZ를 동시에 얻을 수 없습니다. v4는 이 물리적 제한을 숨기지 않고 두 모드로 분리합니다.

- `근접 RGB 전용`: 기본 활성화. 약 100 mm에서 강한 원형의 픽셀 중심과 픽셀 지름만 표시하며 XYZ·실제 지름은 `N/A`입니다.
- `3D 모드`: 근접 RGB 옵션을 끄고 300 mm 이상에서 사용합니다. XYZ를 RGB와 같은 1280×720 해상도로 취득해 작은 홀 주변의 깊이 샘플 손실을 줄였습니다.

로봇 좌표가 필요한 100 mm 작업에서는 고정된 작업 평면과 카메라 외부 보정으로 픽셀을 평면 좌표에 매핑하거나, 100 mm 깊이를 지원하는 ZED Mini/ZED X Mini/근거리 ToF 센서가 필요합니다.

### Roboflow AI 시험

v5는 공개 모델 `hole-detection-fwa4p/2`의 Hosted API를 근접 RGB 검출기에 연결합니다.

1. Roboflow에서 개인 API Key를 발급합니다.
2. 앱의 `Roboflow AI 사용`을 체크합니다.
3. API Key를 비밀번호 입력란에 넣거나, 앱을 실행하는 PowerShell 세션에만 환경변수를 설정합니다.

```powershell
$env:ROBOFLOW_API_KEY="본인의_API_KEY"
.\RUN_LATEST.cmd
```

키는 파일에 저장하거나 화면·로그·CSV에 출력하지 않습니다. 앱을 종료한 뒤 같은 PowerShell 세션에서 `$env:ROBOFLOW_API_KEY=$null`로 제거할 수 있습니다. AI 모드에서는 선택한 ROI가 JPEG로 Roboflow 서버에 전송되며, 호출은 약 1회/초로 제한됩니다. 모델 응답의 `hole` 박스 중심은 기존 5프레임 추적기로 안정화합니다.

이 공개 모델은 내부 테스트 성능이 높아도 현재 ZED/앙카홀 장면에서의 정확도는 검증되지 않았습니다. API Key가 없는 빌드 환경에서는 C# 연동의 컴파일과 로컬 시험만 검증할 수 있고 실제 서버 호출은 검증할 수 없습니다.

## 빌드 및 실행

```powershell
dotnet restore .\ZedAnchorHoleInspection.slnx
dotnet build .\ZedAnchorHoleInspection.slnx -c Release
dotnet run --project .\src\ZedAnchorHoleInspection\ZedAnchorHoleInspection.csproj -c Release
```

합성 데이터 셀프테스트:

```powershell
dotnet run --project .\tests\ZedAnchorHoleInspection.SelfTest\ZedAnchorHoleInspection.SelfTest.csproj -c Release
```

실제 카메라 단일 프레임 테스트:

```powershell
dotnet run --project .\tests\ZedAnchorHoleInspection.SelfTest\ZedAnchorHoleInspection.SelfTest.csproj -c Release -- --camera
```

## 사용 방법

1. ZED Explorer/Depth Viewer/Studio가 카메라를 점유하고 있으면 종료합니다.
2. `LIVE START`를 누릅니다.
3. 영상에서 홀과 주변 평면이 함께 들어오도록 마우스로 ROI를 그립니다.
4. 실제 홀 영상 크기에 맞춰 최소·최대 반지름을 조정합니다.
5. `INSPECTION START`를 누릅니다.
6. 결과의 `X/Y/Z`는 카메라 좌표계 millimeter 단위의 홀 입구 중심입니다.
7. 필요하면 현재 안정 검출 결과를 CSV로 저장합니다.

## 반드시 알아야 하는 한계

- **홀 바닥 XYZ는 보장할 수 없습니다.** 스테레오 대응점이 없으면 깊이는 `N/A`로 표시합니다. `X/Y/Z`는 주변 표면에 투영한 홀 입구 중심입니다.
- 콘크리트 얼룩, 검은 페인트, 원형 볼트 머리도 RGB 후보가 될 수 있습니다. XYZ 평면·내부 검증으로 줄이지만 실제 데이터 없이 완전히 제거할 수 없습니다.
- 홀 가장자리가 깨졌거나 가려졌거나, 카메라가 표면을 매우 비스듬히 보면 원형 점수가 낮아질 수 있습니다.
- 실제 검출률과 치수 정확도는 거리, 조명, 콘크리트 질감, 홀 직경, 베이스라인 방향 및 ZED depth mode에 따라 달라집니다. 현장 RGB+SVO 데이터로 튜닝해야 합니다.
- 로봇 좌표는 출력하지 않습니다. 카메라 XYZ를 로봇 베이스 좌표로 바꾸려면 hand-eye/extrinsic 캘리브레이션이 별도로 필요합니다.
- 안전 관련 로봇 정지·천공 허가는 이 비전 결과 하나만으로 결정하면 안 됩니다. PLC/로봇 측 검증과 안전 인터록이 필요합니다.

## 프로젝트 구조

```text
src/ZedAnchorHoleInspection/
├─ Camera/ZedCamera.cs
├─ Detection/StereoHoleDetector.cs
├─ Detection/TemporalHoleTracker.cs
├─ Models/LiveFrame.cs
├─ MainWindow.xaml
└─ MainWindow.xaml.cs
tests/ZedAnchorHoleInspection.SelfTest/
└─ Program.cs
```
