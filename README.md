# ZED 2i Anchor Hole Inspection

ZED 2i 카메라 영상에서 앙카홀을 검출하고 위치 정보를 표시하는 C# WPF 프로그램입니다.

<img width="2560" height="1552" alt="image" src="https://github.com/user-attachments/assets/46f49558-0cf4-4615-8af7-1e71a2bf6a4c" />
<프로그램 화면>


## 주요 기능

- ZED 2i RGB 영상 실시간 표시
- 마우스 드래그 방식의 검사 ROI 설정
- RGB 원형 특징과 스테레오 깊이를 이용한 홀 후보 검증
- Roboflow `hole-detection-fwa4p/2` 모델을 이용한 AI 검출 옵션
- 홀 주변 평면을 기준으로 한 입구 중심 XYZ 추정
- 연속 프레임 추적으로 검출 결과 안정화
- 검출 조건 조정 및 결과 CSV 저장

## 요구 환경

- Windows 10/11 x64
- .NET 8 SDK 이상
- ZED SDK 5.4.x
- ZED 2i 및 USB 3.x 연결

## 실행

```powershell
.\RUN_LATEST.cmd
```

또는 직접 빌드하고 실행합니다.

```powershell
dotnet build .\ZedAnchorHoleInspection.slnx -c Release
dotnet run --project .\src\ZedAnchorHoleInspection\ZedAnchorHoleInspection.csproj -c Release
```

Roboflow AI를 사용할 때는 앱의 API Key 입력란을 사용하거나 실행 세션에 환경변수를 설정합니다.

```powershell
$env:ROBOFLOW_API_KEY="본인의_API_KEY"
.\RUN_LATEST.cmd
```

## 참고

- ZED 2i는 스테레오 카메라이므로 약 100 mm 근거리에서는 신뢰할 수 있는 깊이 측정이 어렵습니다.
- 근접 작업에서는 RGB/AI로 홀 중심을 검출하고, XYZ는 홀 주변의 유효한 평면 데이터로 추정합니다.
- XYZ는 카메라 로컬 좌표이며 로봇 좌표 변환은 포함하지 않습니다.
- AI 모드에서는 선택한 ROI 이미지가 Roboflow 서버로 전송됩니다.

## License

[MIT](LICENSE)
