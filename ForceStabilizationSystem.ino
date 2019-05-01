/*
 Name:		ForceStabilizationSystem.ino
 Created:	30.05.2017 21:31:45
 Author:	Kernel Plevitsky
*/

#include <NewPing.h>
#include <Servo.h>
#include <FlexiTimer2.h>
#include <string.h>
#include <math.h>



//CALIBRATION
#define STARTANGLE 60				// Калибровать! Установить угол сервопривода при инициализации системы
#define STABSTATEFORCE 139			// Калибровать! Показания датчика [ объект в центре платформы ]
#define MINSENS 4					// Калибровать! Показания датчика [ минимальный показатель ]
#define MAXSENS 200					// Калибровать! Показания датчика [ максимальный показатель ]
#define SERVOMINANGLE 0			    // Калибровать! Минимальный(начальный) угол сервопривода [ рабочий диапазон ]
#define SERVOMAXANGLE 130			// Калибровать! Максимальный(финальный) угол сервопривода [ рабочий диапазон ]
#define ANGLEFACTOR 6.0			    // Калибровать! (желательно вручную) Коэффициент для расчета изменения угла
#define SERVODIRECT 1				// Калибровать! (вручную) Ориентация сервопривода
#define SENSCONNPIN A0				// Подключение датчика
#define SENSCONPIN_TRIG 7			// Триггер ультразвукового датчика
#define SENSCONPIN_ECHO 6			// Эхо ультразвукового датчика
#define SERVOCONNPIN 3				// Подключение сервопривода
#define SPEEDOPMS 1.0				// секунды. Делимое задержки. [ только для режима АЦП с тензодатчиком ]
#define SPEEDOPSUB 1000				// Делитель. Кол-во операций выполняемых за SPEEDOPMS. По-умолчанию "1.0\1000" = одна операция за  1с. [ только для режима АЦП с тензодатчиком ]
									// Для ультразвукового датчика максимальная частоста "1.0\20"

//new
#define SYSSPDREACT 100				// .мс  Скорость реакции системы на воздействие окружения
#define STABSTATEFORCE_RANGE 15	    // Калибровать! Диапазон погрешности границ. По-другому отступ от границ
#define STABSTATEFORCE_LEFT 18		// Калибровать! Показания датчика [ объект на границе слева ]
#define STABSTATEFORCE_RIGHT 264    // Калибровать! Показания датчика [ объект на границе справа ]
#define INTERFERENCE_LOW 5          // Калибровать! Уровень отсекаемых помех при снятии показателей с датчика
#define INTERFERENCE_HIGHp 4
#define INTERFERENCE_HIGHn -4
#define RIGHTMOVEMULT 1.5			// Калибровать! Множитель коэффициента. При ультразвуковом отслеживании объекта, отдаление считается хуже, поэтому необходимо усилить коэффициент.

//Указать используемый тип датчика

//types: TENSO | USOUND
#define USOUND 1
#define TENSO 2
#define SENSOR_TYPE USOUND

// Использование режима АЦП. ps: Analog-Digital-Converter
#define FASTADC 1				

//Служебные макросы
#define GO true
#define fp(str) Serial.println(str)   //FAST PRINT
#define fp_(str) Serial.print(str)    //FAST PRINT without new line

// установка и снятие бита с регистра
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
//#Служебные макросы

//***SERVO DIRECTION***
//
//   >  0-90-180     1
//   <  180-90-0     0
//
//********END**********




class Control
{
public:
	static const int SensorPin = SENSCONNPIN;
	static const int StartAngle = STARTANGLE;
	static const int ServoPin = SERVOCONNPIN;
	static const int ServoDirection = SERVODIRECT;
	static const int ServoMinAngle = SERVOMINANGLE;
	static const int ServoMaxAngle = SERVOMAXANGLE;
	static const int SpeedOperationMs = SPEEDOPMS;
	static const int SpeedOperationSub = SPEEDOPMS;
	static const int StabilityStateForce = STABSTATEFORCE;
	static const int StabilityStateForceRange = STABSTATEFORCE_RANGE;
	

	static Servo Srv;
	static NewPing sonar;
	static long force1;
	static long force2;
	static long delta;
	static double factor;
	static int CurrentAngle;
	static double AngleFactor; 
	static bool running;

	static void GetDeltaSpeed(int val = -1) {
		int tmpDelta = 0;

		if (force1<0) force1 = (val < 0) ? analogRead(SensorPin) : val;
		else
		{
			force2 = (val < 0) ? analogRead(SensorPin) : val;
			tmpDelta = force1 - force2;

			if ((tmpDelta >= -INTERFERENCE_LOW) && (tmpDelta <= INTERFERENCE_LOW)) delta = 0;
			else delta = tmpDelta;
			force1 = -1;
		}
		factor = delta * Control::AngleFactor;
		if (delta != 0) running = true;
		else running = false;
	}
	static void GetDeltaSpeedUltraSound() {
		long tmpDelta = 0;
		long duration = 0, cm = 0;

		digitalWrite(SENSCONPIN_TRIG, LOW);
		delayMicroseconds(5);
		digitalWrite(SENSCONPIN_TRIG, HIGH);
		delayMicroseconds(10);
		digitalWrite(SENSCONPIN_TRIG, LOW);
		pinMode(SENSCONPIN_ECHO, INPUT);
		duration = pulseIn(SENSCONPIN_ECHO, HIGH);
		cm = duration; //(duration / 2) / 29.1 если в сантиметрах

		if (force1<0) force1 = cm / 10;
		else
		{
			force2 = cm / 10;
			tmpDelta = force1 - force2;
			if ((tmpDelta >= -INTERFERENCE_LOW) && (tmpDelta <= INTERFERENCE_LOW)) delta = 0;
			else delta = tmpDelta;

			force1 = -1;
		}
		
		

		factor = delta * Control::AngleFactor;
		if (factor < 0) factor *= RIGHTMOVEMULT;
		if (delta != 0) running = true;
		else running = false;
	}

	static void Setup() {
		force1 = -1;
		force2 = -1;
		delta = 0.0;
		factor = 0.0;
		CurrentAngle = 0;
		AngleFactor = ANGLEFACTOR;

		Srv.attach(ServoPin);
		Srv.write(StartAngle);

		Serial.begin(115200);
		
		#if SENSOR_TYPE == 2
			FlexiTimer2::set(1, SpeedOperationMs / SpeedOperationSub, Control::GetDeltaSpeed); //every 1ms call GetSpeed()
			FlexiTimer2::start();
		#else
			pinMode(SENSCONPIN_TRIG, OUTPUT);
			pinMode(SENSCONPIN_ECHO, INPUT);
			NewPing::timer_ms(50, Control::GetDeltaSpeedUltraSound);
		#endif

	}
	//Функция безопасна т.к. не применяет угол выходящий за рабочий диапазон
	static int ServoChange(int angle, bool accept=false)
	{
		//ServoChange(12)       - узнать какой угол будет после прибавления 12 градусов
		//ServoChange(12, true) ~ + применить этот угол
		int tmpAngle = Srv.read();

		tmpAngle += angle;

		if ((accept) && (tmpAngle < ServoMaxAngle) && (tmpAngle > ServoMinAngle))
		{
			Srv.write(tmpAngle);
			return tmpAngle;
		}
		else return tmpAngle;
	}

	//Функция корректирующая угол сервопривода в соответствии с движением объекта на платформе
	static void CorrectAngle() {

		CurrentAngle = Srv.read();

		if (((CurrentAngle - factor) > ServoMaxAngle) || ((CurrentAngle - factor) < ServoMinAngle))
		{
			fp_("Коэффициент: "); fp(factor);
			fp("Внимание! Коэффициент расчета угла достиг критических значений! Процедура корректировки угла проигнорирована!");
			return;
		}

		if (ServoDirection == 1) { CurrentAngle -= factor; }
		else { CurrentAngle += factor; }
		
		
		if (running) { Srv.write(CurrentAngle); fp_("Угол скорректирован на: "); fp(CurrentAngle); }
		else { fp_("Объект не движется - показания: "); fp(delta); }
		
	}
	
	static void CorrectPosition()
	{
		// Функция будет дестабилизировать объект до тех пор, пока он не окажется в нужной зоне,
		// постепенно наклоняя платформу в нужном направлении, до тех пор, пока объект не обретет
		// ускорение.
		// Если объект будет находится в нужной зоне. Функция не будет применена.
		
		if (running) return;
		else if (running) return;

		if (force2 < (StabilityStateForce-StabilityStateForceRange)) //left unstable side
		{
			while (!running)
			{
				ServoChange(-2, GO);
				delay(50);
				fp("Дестабилизация >>>>>>>>!");
			}
		}
		else if (force2 > (StabilityStateForce + StabilityStateForceRange)) //right unstable side
		{
			while (!running)
			{
				ServoChange(2, GO);
				delay(50);
				fp("Дестабилизация <<<<<<<<!");
			}
		}
		else { fp("Объект стабилизирован в необходимой области. Корректировка в пространстве завершена!"); }
	}

	static int clrDelta()
	{
		int Result = 0;

		if ((force2 > STABSTATEFORCE)&&(delta > 0)) Result = delta * (-1);
		else if ((force2 < STABSTATEFORCE) && (delta < 0)) Result = delta * (-1);
		else return delta;

		return Result;
	}

	static void GetDebugInf() {
		
		fp_("SPD: "); fp_(delta); 
		fp_(" WGT:");  fp_(force2);
		if (factor < 0) fp_(" >>>>>>>>");
		else if (factor > 0) fp_(" <<<<<<<<");

		else fp_(" STOPPED"); 

		fp_(" AGL: "); fp(factor);
	}

	/* Функция калибровки - нахождение коэффициента зависимости поворота угла серпопривода
	   от скорости движения объекта. Функция рекурсивна. Возможны весьма резкие повороты
	   сервопривода! Для ручной калибровки закомментируйте функцию помеченную "??"        
	*/
	static void Calibrating_ANGLEFACTOR_ot(int Step = 1, int _angle = SERVOMINANGLE, int _tt = 0)
	{//проверить, что до вызова функции калибровки был установлен стартовый угол платформы
		int tt = _tt;				//Кол-во градусов добавленных к углу сервопривода
		int vv = 0;					//Начальная скорость объекта при обнаружении движения
		double zz = 0;			    //Коэффициент

		fp("Поставьте объект в начало платформы и установите ей нестабильный угол, затем перезапустите систему!");
		fp("Дождитесь, пока объект стабилизируется. Возможно резкое поведение сервопривода!");
		
		Srv.write(STARTANGLE + 40);
		delay(8000);
		Srv.write(_angle);

		while (running==false) { fp("стоим"); };

		fp("движемся");

		vv = delta; 

		while (delta != 0)
		{
			ServoChange(Step, GO);
			tt++;
			delay(SYSSPDREACT);
			fp("угол изменен");
		}

		zz = (double)tt / (double)vv;

		if ((force2 >= STABSTATEFORCE_RIGHT - STABSTATEFORCE_RANGE) || (force2 <= STABSTATEFORCE_LEFT + STABSTATEFORCE_RANGE))
		{// погрешность границ +- 5. желательно проверить

			fp("Система не успела рассчитать угол остановки! Повторите калибровку с полученным значением!");
			fp_("Использовано градусов: "); fp(tt);
			fp_("Текущий коффициент угла: "); fp(zz);

			//?? закомментировать функцию, если нужен ручной одноразовый калибровочный проход
			Calibrating_ANGLEFACTOR_ot(1, _angle + tt, tt);

			return;
		}

		fp_("Поздравляем! Коэффициент угла, успешно рассчитан и равен "); fp(zz);
		fp_("Использовано градусов: "); fp_(tt); fp_(", при delta= "); fp(vv);

		//возможные проблемы:
		// 1) Коэффициент может вычисляться быстрее, чем сервопривод изменяет угол
		// 2) Калибровку придется повторить, если она завершилась до полной остановки объекта
	}
};

//GLOBALS
Servo Control::Srv;
NewPing Control::sonar(SENSCONPIN_TRIG, SENSCONPIN_ECHO, MAXSENS);
long Control::force1;
long Control::force2;
int Control::CurrentAngle;
double Control::AngleFactor;
long Control::delta;
double Control::factor;
bool Control::running = false;

//END

// code
Control ctrl;



void setup() {
#if FASTADC
	// set prescale to 16
	sbi(ADCSRA, ADPS2); // Установка режима AVR. Делитель частоты 16
	cbi(ADCSRA, ADPS1); // 1 0 0 = 16
	cbi(ADCSRA, ADPS0); //
#endif

	Control::Setup();


	/* !!! ПОСЛЕ УСПЕШНОЙ КАЛИБРОВКИ - ЗАККОМЕНТИРОВАТЬ СТРОКУ НИЖЕ !!! */
	//Control::Calibrating_ANGLEFACTOR_ot();

}

void loop() {
	//ctrl.GetDebugInf();
	//fp_("------------------------------------DELTA "); fp(ctrl.delta);

	ctrl.CorrectAngle();
	ctrl.CorrectPosition();


	delay(SYSSPDREACT);
}
