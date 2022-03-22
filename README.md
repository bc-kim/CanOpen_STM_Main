# CanOpen_STM_Main

## **Update Log**
### **2022.03.22**

**[CAN_SendDesiredValue함수수정3] 커밋**

- CAN_SendDesiredValue함수 내에서 CAN_SendPDO가 Node가 Node-1이 되어있어서 Node4에 값이 포함안되던 문제 해결
- 아마 밑에 있는 모터드라이버 4번에 원하는 값이 안들어가는 문제가 해결되지 않을까?
- 변수 잘못 적은거 수정해서 뒤에 2 붙임 (DesiredValuePreset_M4)
- 변수 잘못 적은거 수정해서 뒤에 3 붙임, 일단 4번에 원하는 값이 들어가지만 j가 증가하지않음을 확인.

**[Con_ModePresetM# 추가] 커밋**

- 각 모터 드라이버에 다른 ConMode, DesiredValue 배열이 입력되도록 변경.
- j=1커밋에서 모터드라이버 4번에만 원하는 값이 들어가지 않음을 확인함.

**[j=1일때 if문 예외 추가] 커밋**

- j=1일때 CON_Mode와 CON_Mode_Prev가 같아서 CON_Mode가 제대로 설정 안되던 문제 해결
- 아직 [2022_0321_다시짠코드_연속적으로TargetValue갈수있도록] 커밋 검증은 안됐음. 

### **2022.03.21**

**[2022_0321_다시짠코드_연속적으로TargetValue갈수있도록] 커밋**

- 아직 검증이 안됐음.
- Target Value를 배열로 입력하면 그 값들 차근차근 갈수 있도록 짰는데 확인이 필요함.

**[ControlModePreset/DesiredForcePreset/DesiredValuePreset추가2] 커밋**

- Target값들을 그냥 선언하면 안되고 memcpy로 해야하는 것 같아서 memcpy로 작성
- memcpy(&motor_[i].Con_Mode[0], &Con_ModePreset[0], 40); 이런식으로 짬.

**[220321_기존 실험 코드 업로드] 커밋**

- 지난 실험에서 검증이 됐는데 Target Value를 하나만 갈수 있도록 해서 위에 수정 새로 했음. -> 검증하고 파기하면 될 것 같음.


-----------

