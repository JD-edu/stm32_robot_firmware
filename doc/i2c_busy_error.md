
STM32에 MPU6050을 연결해서 테스트 중인데 아무래도 MPU6050을 시작할 때 리셋을 한번 해야 할 것 같아요 리셋하는 코드를 보여주세요

MPU6050을 **리셋(Reset)**하려면 PWR_MGMT_1 레지스터(0x6B)의 RESET 비트를 설정하면 됩니다.
---

📌 1. MPU6050 리셋 방법

MPU6050의 0x6B (PWR_MGMT_1) 레지스터에 0x80을 쓰면(RESET 비트 1 설정) 전체 칩이 리셋됩니다.
✅ 리셋 과정
1. MPU6050의 RESET 비트(0x80) 설정 (PWR_MGMT_1 → 0x6B)
2. MPU6050이 리셋되는 동안 약 100ms 대기
3. 다시 PWR_MGMT_1을 0x00으로 설정하여 일반 동작 모드로 전환
4. 다른 설정(가속도계, 자이로 설정) 다시 초기화
---

📌 2. STM32에서 MPU6050 리셋 코드

```
#include "main.h"

#define MPU6050_ADDR 0x68 << 1  // 7비트 주소를 << 1 하여 8비트 주소로 변환
#define PWR_MGMT_1  0x6B        // 전원 관리 레지스터

extern I2C_HandleTypeDef hi2c1; // CubeMX에서 설정한 I2C 핸들

// I2C를 이용한 MPU6050 레지스터 쓰기 함수
void MPU6050_Write(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buffer, 2, 100);
}

// MPU6050 소프트 리셋 함수
void MPU6050_Reset() {
    // 1️⃣ PWR_MGMT_1 레지스터(0x6B) 리셋 비트(0x80) 설정
    MPU6050_Write(PWR_MGMT_1, 0x80);
    
    // 2️⃣ MPU6050 리셋 대기 (100ms)
    HAL_Delay(100);

    // 3️⃣ PWR_MGMT_1을 0x00으로 설정하여 일반 모드로 변경
    MPU6050_Write(PWR_MGMT_1, 0x00);
    
    // 4️⃣ 추가 설정 (선택 사항)
    MPU6050_Write(0x1A, 0x03);  // CONFIG 레지스터 (DLPF 활성화)
    MPU6050_Write(0x1B, 0x00);  // 자이로스코프 범위 설정 (±250°/s)
    MPU6050_Write(0x1C, 0x00);  // 가속도계 범위 설정 (±2g)

    HAL_Delay(50);  // 추가 안정화 대기
}

```
✅ 이제 MPU6050_Reset();을 호출하면 MPU6050이 리셋됩니다.
✅ 리셋 후 추가 설정을 다시 해야 함 (가속도계, 자이로스코프 설정 등)
---

📌 3. main.c에서 MPU6050 리셋 적용

```
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();  // I2C 초기화

    MPU6050_Reset();  // MPU6050 리셋 실행

    while (1) {
        // 이후 MPU6050 데이터를 읽거나 사용할 수 있음
    }
}

```

---

📌 4. 리셋이 필요한 경우

✅ MPU6050이 정상적으로 동작하지 않을 때
✅ MPU6050이 I2C 응답을 하지 않을 때
✅ 전원이 불안정하여 MPU6050이 비정상적으로 동작할 때
---

📌 5. 결론

1️⃣ PWR_MGMT_1 레지스터(0x6B)에 0x80을 쓰면 MPU6050이 리셋됨
2️⃣ 리셋 후 100ms 대기 필요
3️⃣ MPU6050 초기 설정(자이로, 가속도계, DLPF 등)을 다시 해야 함
4️⃣ STM32에서는 HAL_I2C_Master_Transmit()을 이용해 리셋 가능
5️⃣ MPU6050_Reset(); 함수를 호출하여 간단하게 리셋 가능
이제 STM32에서 MPU6050을 정상적으로 리셋하고 초기화할 수 있습니다! 🚀
추가 질문이 있으면 언제든지 알려주세요! 😊

/* Wait until BUSY flag is reset */ if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY_FLAG, tickstart) != HAL_OK) { return HAL_BUSY; } 여기서 에러가 생기고 있습니다. 어떤 원인일까요?

I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY_FLAG, tickstart) != HAL_OK
이 부분에서 I2C가 BUSY 상태에서 빠져나오지 못하고 타임아웃이 발생하는 문제가 발생하고 있습니다.
---

📌 1. I2C_FLAG_BUSY 에러 원인

STM32에서 I2C_FLAG_BUSY 플래그는 I2C 버스가 사용 중일 때 SET(1) 상태가 됩니다.
이 상태에서 계속 BUSY 상태가 유지되면 통신을 시작할 수 없어 타임아웃이 발생하게 됩니다.
✅ 주요 원인:
1️⃣ MPU6050이 I2C 응답을 하지 않음 (전원이 꺼짐, 불량, 연결 오류 등)
2️⃣ 이전 I2C 통신이 비정상 종료됨 (클럭 스트레칭, NACK 오류 등)
3️⃣ I2C SDA/SCL 핀이 LOW 상태로 유지됨 (하드웨어 LOCK-UP 상태)
4️⃣ MPU6050이 리셋되지 않아 I2C 통신이 막힘
5️⃣ STM32의 I2C 모듈이 초기화되지 않음 또는 잘못 초기화됨
---

📌 2. 문제 해결 방법


✅ 1️⃣ I2C 버스가 점유된 경우 강제 해제 (I2C 재설정)

I2C가 BUSY 상태에서 벗어나지 못하면 I2C를 소프트 리셋해야 합니다.

🔹 해결 코드 (I2C 소프트 리셋)

```
void I2C_Reset(I2C_HandleTypeDef *hi2c) {
    __HAL_RCC_I2C1_FORCE_RESET();  // I2C1을 강제로 리셋
    HAL_Delay(10);
    __HAL_RCC_I2C1_RELEASE_RESET(); // I2C1 리셋 해제
}

```
이제 main()에서 I2C가 BUSY 상태이면 강제로 리셋할 수 있습니다.
```
if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) {
    I2C_Reset(&hi2c1);
}

```
✅ 이 방법은 I2C가 비정상적으로 멈췄을 때 강제 해결하는 방법입니다.
---

✅ 2️⃣ I2C 핀을 강제로 풀업 (SDA/SCL 초기화)

하드웨어 LOCK-UP 상태라면 SDA, SCL 핀을 직접 HIGH로 만들고 I2C를 재초기화해야 합니다.

🔹 해결 코드 (SDA/SCL 핀 초기화)

```
void I2C_BusRecover(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* SDA & SCL을 GPIO로 설정 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9; // I2C1_SCL (PB8), I2C1_SDA (PB9)
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SDA를 HIGH로 만든 후 SCL을 9번 토글하여 클럭을 생성 */
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);  // SCL HIGH
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // SCL LOW
        HAL_Delay(1);
    }

    /* 다시 I2C 모드로 변경 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

```
✅ STM32에서 I2C가 LOCK-UP된 경우 I2C를 복구할 수 있습니다.
---

✅ 3️⃣ MPU6050을 리셋 후 I2C 다시 초기화

MPU6050이 응답하지 않으면 MPU6050을 소프트 리셋한 후 다시 시도해야 합니다.

🔹 MPU6050 리셋 코드

```
void MPU6050_Reset() {
    uint8_t reset_cmd = 0x80;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &reset_cmd, 1, 100);
    HAL_Delay(100);
}

```
✅ MPU6050이 BUSY 상태라면 이 방법으로 해결 가능!
---

✅ 4️⃣ MPU6050의 전원을 끈 후 다시 켜기

MPU6050이 완전히 멈춰서 I2C 응답을 하지 않는다면 전원을 껐다가 다시 켜야 합니다.

🔹 해결 방법

1. MPU6050을 GPIO로 제어하는 경우:
   
2. 전원이 꺼지지 않는 경우: 손으로 직접 전원을 껐다가 켜보세요.
✅ MPU6050이 전원 문제로 응답하지 않는 경우 효과적인 방법입니다.
---

📌 3. 최종 해결 코드

```
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    /* I2C BUSY 문제 해결 */
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) {
        I2C_Reset(&hi2c1);
        I2C_BusRecover();  // I2C SCL 9번 토글
        HAL_Delay(100);
        MX_I2C1_Init();  // I2C 다시 초기화
    }

    /* MPU6050 리셋 */
    MPU6050_Reset();
    
    while (1) {
        // MPU6050 데이터 읽기
    }
}

```

---

📌 결론: 가장 효과적인 해결 방법

✅ 1️⃣ I2C 강제 리셋 (I2C_Reset())
✅ 2️⃣ SDA/SCL을 강제로 HIGH 후 9번 SCL 펄스 (I2C_BusRecover())
✅ 3️⃣ MPU6050 소프트 리셋 (MPU6050_Reset())
✅ 4️⃣ 필요하면 전원을 껐다가 다시 켜기
이제 STM32에서 I2C BUSY 문제를 해결하고 MPU6050을 정상적으로 초기화할 수 있습니다! 🚀
추가 질문이 있으면 언제든지 알려주세요! 😊
