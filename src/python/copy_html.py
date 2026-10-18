Import("env")
import os, shutil
import elrs_helpers

os.chdir('html')
target_name = env['PIOENV'].upper()
build_flags = env.get('BUILD_FLAGS', [])

target_type = elrs_helpers.get_target_type(target_name, build_flags)
type = 'tx' if target_type == 'TX' else 'rx'
is8285 = '-8285' if ('ESP8285' in target_name or env.get('PIOPLATFORM', '') == 'espressif8266' or any('PLATFORM_ESP8266' in str(f) for f in build_flags)) else ''
chip = 'sx128x' if '-DRADIO_SX128X=1' in build_flags else 'sx127x' if '-DRADIO_SX127X=1' in build_flags else 'lr1121' if '-DRADIO_LR1121=1' in build_flags else 'lr2021' if '-DRADIO_LR2021=1' in build_flags else ''

is_eml = any('EML_BRIDGE' in str(f) for f in build_flags)
shutil.copy('headers/web-eml-bridge.h' if is_eml else f'headers/web-{chip}-{type}{is8285}.h', '../include/WebContent.h')
