# 🤖 Self-Balancing Robot (STM32)

<p align="center">
  <img src="balancin.gif" width="400"/>
</p>

🚀 Robot autobalanceado en tiempo real utilizando control PID y fusión de sensores.

---

## 🔧 Build del sistema

<p align="center">
  <img src="build.png" width="600"/>
</p>

---

## 🧠 Descripción

Este proyecto consiste en el desarrollo de un robot tipo “balancín” capaz de mantenerse en equilibrio sobre dos ruedas mediante técnicas de control clásico. El sistema combina adquisición de datos en tiempo real, fusión de sensores y control dinámico para estabilizar el ángulo del sistema.

El enfoque principal fue lograr un control robusto frente a perturbaciones, optimizando el comportamiento mediante ajustes de PID y filtrado de señales.

---

## ⚙️ Tecnologías utilizadas

- Lenguaje: C  
- Microcontrolador: STM32 (Black Pill - STM32F411)  
- Sensores: MPU6050 (acelerómetro + giroscopio)  
- Comunicación: I2C, UART/WiFi, USB CDC  
- Control: PID en tiempo real  
- Interfaz: aplicación en Qt para visualización de datos  

---

## 🚀 Características principales

- Estabilización en tiempo real del sistema  
- Control PID ajustable  
- Fusión de sensores (complementary filter)  
- Filtrado de señales (low-pass)  
- Manejo preciso del tiempo de muestreo (dt)  
- Comunicación con PC para monitoreo en vivo  
- Visualización de variables del sistema  
- Registro de datos (CSV y WiFi)  

---

## 📊 Variables monitoreadas

- Ángulo del sistema (roll)  
- Error de control  
- Términos PID (P, I, D)  
- Señales PWM (comando y saturación)  
- Datos del MPU6050 (crudos y filtrados)  
- Tiempo de ejecución del loop  

---

## 🖥️ Interfaz gráfica

<p align="center">
  <img src="interfaz_QT.png" width="700"/>
</p>

El sistema incluye una aplicación desarrollada en Qt que permite:

- Visualizar datos en tiempo real  
- Analizar el comportamiento del sistema  
- Ajustar parámetros de control  
- Registrar datos para análisis posterior  

---

## 🧪 Desafíos técnicos

- Ajuste del control PID para evitar inestabilidades  
- Manejo del tiempo de muestreo (dt) y jitter  
- Filtrado de señales ruidosas  
- Integración de sensores, control y comunicación  
- Control de saturaciones y efecto del término integral  

---

## 🎯 Objetivos del proyecto

- Desarrollar un sistema de control robusto en tiempo real  
- Integrar hardware y software en una solución funcional  
- Aplicar conceptos de control en un sistema físico real  
- Mejorar la estabilidad frente a perturbaciones  

---

## 🔧 Posibles mejoras

- Implementación de control LQR o adaptativo  
- Uso de filtros más avanzados (Kalman)  
- Optimización del rendimiento  
- Mejora de estabilidad en movimiento  

---

## 👨‍💻 Autor

Tadeo Mendelevich  
https://github.com/tadeomendelevich
