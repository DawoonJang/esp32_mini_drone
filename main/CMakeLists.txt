idf_component_register(
    SRCS "main.c" "src/motor.c" "src/mpu9250.c"  # 모든 소스 파일을 한 번에 등록
    INCLUDE_DIRS "." "src"  # 현재 디렉터리 및 src 폴더 포함
    REQUIRES driver freertos  # 필요한 의존성 추가
)