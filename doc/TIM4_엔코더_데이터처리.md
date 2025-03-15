STM32에 모터 엔코더를 연결해서 엔코더 값을 받으려고 한다면 어떤 프로세스를 거처야 하는가?

STM32에서 TIM4를 이용하여 모터 엔코더를 인터페이스하는 방법을 설명하고, 예제 코드를 제공하겠습니다.
STM32의 TIMx 타이머는 엔코더 인터페이스 모드를 지원하므로, 이를 활용하여 모터의 엔코더 값을 읽을 수 있습니다.
---

📌 1. 엔코더 모드 설정 방법 (TIM4)

STM32의 TIM4 타이머를 엔코더 인터페이스 모드로 설정하려면 다음 단계를 따릅니다.

1.1 하드웨어 연결

- 모터 엔코더의 A 채널 → TIM4_CH1 (예: PB6)
- 모터 엔코더의 B 채널 → TIM4_CH2 (예: PB7)
이 설정은 STM32F407 기준입니다. 사용 중인 MCU의 TIM4_CH1, TIM4_CH2 핀을 확인하세요.

1.2 STM32CubeMX 설정

1. TIM4 활성화
	- Mode → Encoder Mode → Encoder Mode TI1 and TI2
	- Counter Period → 65535 (16비트 최대값)
	- Prescaler → 0 (1:1 분주비)
2. 입력 채널 설정 (Channel 1 & Channel 2)
	- Polarity: Rising 또는 Both Edges
	- Filter: 10 정도 (노이즈 방지)
3. 클럭 활성화
	- RCC에서 TIM4 타이머 활성화
4. GPIO 설정
	- PB6 (TIM4_CH1), PB7 (TIM4_CH2)를 Alternate Function으로 설정
	- Pull-up 또는 Pull-down 활성화
---

📌 2. 코드 예제 (HAL 라이브러리 사용)

아래 코드는 TIM4를 이용하여 엔코더 데이터를 읽는 예제입니다.

2.1 main.c

```
#include "main.h"

TIM_HandleTypeDef htim4;
volatile int32_t encoder_count = 0;  // 엔코더 값 저장 변수

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM4_Init();

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // 엔코더 인터페이스 시작

    while (1)
    {
        encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    }
}

static void MX_TIM4_Init(void)
{
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535; // 16-bit max
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.RepetitionCounter = 0;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    TIM_Encoder_InitTypeDef sConfig = {0};
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 10;

    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 10;

    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();  // GPIOB 클럭 활성화

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

```

---

📌 3. 코드 설명

1. MX_TIM4_Init()
	- TIM4를 Encoder Mode로 설정
	- IC1과 IC2를 각각 Rising Edge 감지로 설정
	- Filter = 10으로 노이즈 방지
2. HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL)
	- TIM4을 엔코더 모드로 동작 시작
3. __HAL_TIM_GET_COUNTER(&htim4)
	- 현재 엔코더 값을 읽음
---

📌 4. 엔코더 값 해석

- 기본적으로 TIM4는 16비트 카운터(0~65535)를 사용합니다.
- 카운터가 **최대값(65535)**을 넘어가면 0으로 돌아가고, 0에서 마이너스 값이 되면 65535로 돌아갑니다.
- 엔코더 해석 공식:
  
	- encoder_position: 현재 엔코더 값
	- velocity: 속도 (이전 값과 현재 값의 차이)
---

📌 5. 오버플로우 처리

만약 16비트 카운터(0~65535)의 한계를 넘어서면 HAL_TIM_PeriodElapsedCallback()을 사용하여 보정할 수 있습니다.
```
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        if (__HAL_TIM_GET_COUNTER(htim) > 32767)
            encoder_count -= 65536;
        else
            encoder_count += 65536;
    }
}

```
이렇게 하면 32비트 int32_t 변수로 오버플로우 없이 엔코더 데이터를 받을 수 있습니다.
---

📌 6. 결론

STM32에서 TIM4를 사용하여 모터 엔코더 값을 읽는 방법을 설명했습니다.
위 코드를 실행하면 엔코더의 A/B 채널 입력에 따라 TIM4의 카운터 값이 증가/감소하며 모터의 위치를 추적할 수 있습니다.
추가적으로 속도 계산, 방향 판별, 필터링 등을 적용하면 더욱 정밀한 로봇 제어가 가능합니다! 🚀
