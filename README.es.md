# Convertidor de Trinet a MCAP

[![Language: English](https://img.shields.io/badge/Language-English-blue.svg)](README.md)

Una aplicación de Python independiente para convertir grabaciones de cámara Trinet egocéntricas (MP4 con datos IMU SEI incrustados) en archivos MCAP comprimidos con LZ4 compatibles con Foxglove Studio.

## Características
- **Extracción de Datos**: Divide automáticamente los archivos MP4 de Trinet en video limpio y archivos binarios IMU/VTS.
- **Conversión a Annex B**: Convierte el flujo H.264 de AVCC (MP4) a formato Annex B para decodificación fluida en Foxglove.
- **Pose de Cabeza 3D**: Calcula la orientación en tiempo real usando el filtro Madgwick (soporta 6-DOF y 9-DOF).
- **Estándares de Robótica**:
    - **Árbol TF**: `world` -> `imu` -> `cam0`.
    - **Intrínsecos**: Publicados vía `foxglove.CameraCalibration` (modelo Kannala-Brandt).
    - **IMU**: Datos de alta frecuencia para velocidad angular, aceleración lineal y campo magnético.

## Requisitos Previos
- Python 3.8+
- `ffmpeg` y `ffprobe` instalados en el PATH del sistema.

## Instalación
1. Activa tu entorno virtual (asegúrate de que esté cargado).
2. Instala las dependencias necesarias:
   ```bash
   pip install -r requirements.txt
   pip install ahrs scipy
   ```

## Uso
Ejecuta el script de conversión con los siguientes argumentos:
```bash
python convert.py --input sample_data/clothes.mp4 --calibration sample_data/calibration.json --output sample_output/output.mcap [OPCIONES]
```

**Opciones:**
- `--use-mag`: Habilita la fusión 9-DOF usando el magnetómetro (Por defecto).
- `--no-mag`: Deshabilita el magnetómetro para el cálculo de la pose (usa fusión 6-DOF).

## Tópicos de Datos
| Tópico | Esquema | Descripción |
| --- | --- | --- |
| `/camera/image/compressed` | `foxglove.CompressedVideo` | Video H.264 en formato Annex B. |
| `/camera/calibration` | `foxglove.CameraCalibration` | Intrínsecos fisheye (Kannala-Brandt). |
| `/imu/angular_velocity` | `foxglove.Vector3` | Datos de giroscopio corregidos. |
| `/imu/linear_acceleration` | `foxglove.Vector3` | Datos de acelerómetro corregidos. |
| `/imu/magnetic_field` | `foxglove.Vector3` | Datos crudos del magnetómetro. |
| `/tf` | `foxglove.FrameTransform` | Transformaciones Dinámicas (`world->imu`) y Estáticas (`imu->cam0`). |

## Extrínsecos y Árbol TF
El convertidor construye un sistema de coordenadas jerárquico (Árbol TF) para representar el movimiento de la cámara en el espacio 3D. Mientras que el tópico `/camera/calibration` proporciona matrices de proyección estándar para visualización, los extrínsecos físicos se manejan de la siguiente manera:
1. **`world` -> `imu`**: Una transformación dinámica que representa la pose de la cabeza, calculada mediante el filtro de fusión Madgwick de 9 ejes.
2. **`imu` -> `cam0`**: Una **transformación estática que contiene los extrínsecos crudos**.
    - La submatriz de rotación `R_cam_imu` se convierte en un cuaternión unitario ($x, y, z, w$).
    - El vector de traslación `t_cam_imu_m` se mapea directamente como un vector 3D.
    - Esto sigue el protocolo estándar requerido, permitiendo que cualquier pipeline posterior consulte la posición exacta de la cámara respecto al IMU a través del tópico **`/tf`**.

## Archivado y Metadatos
Para garantizar la procedencia total de los datos y el acceso a los valores numéricos originales:
- El contenido íntegro del archivo `calibration.json` de entrada se guarda como **metadatos globales de MCAP** bajo el nombre `calibration_json`.
- Esto archiva todas las matrices y parámetros originales antes de cualquier conversión o formateo.
