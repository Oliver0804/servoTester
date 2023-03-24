## 這是一個用於測試舵機的Arduino程式，具有手動模式、置中模式和自動模式，方便使用者測試舵機的功能。

### 功能
1. 手動模式：通過旋鈕操縱舵機轉動。
2. 舵機置中：將舵機轉動至90度，使其處於中心位置。
3. 自動模式：舵機自動在最小角度和最大角度之間來回轉動。
### 硬體需求
1. Pico 開發板
2. 舵機 (Servo)
3. 按鈕 (Button)
4. 可變電阻 (Potentiometer)
5. LED x 3 (用於顯示當前模式)
6. 麵包板及杜邦線
### 接線說明

![具體引腳除了ADC以外沒其他要求，可自行調整"](https://github.com/Oliver0804/servoTester/blob/main/pico.png)

"具體引腳除了ADC以外沒其他要求，可自行調整"

舵機	-	servoPin (18)
按鈕	-	buttonPin (15)
旋鈕	-	A0 (26)
手動模式 LED	-	manualLED (5)
置中模式 LED	-	centerLED (6)
自動模式 LED	-	autoLED (7)
### 使用說明
將Arduino程式上傳至開發板。
連接硬體並按照接線說明進行接線。
確保舵機接線正確，否則可能無法正確控制舵機。
按下按鈕切換模式，每按一次按鈕，模式將按照以下順序切換：手動模式 → 置中模式 → 自動模式 → 手動模式。
觀察LED燈的狀態，了解當前處於哪種模式。
### 注意事項
確保使用的舵機與程式設定的角度範圍相符，否則可能無法正常工作。
請勿在舵機運行過程中強行擾動舵機，以免損壞舵機。
### 支援
如遇到任何問題，請在此專案的問題區提出，我們將竭誠為您提供幫助。