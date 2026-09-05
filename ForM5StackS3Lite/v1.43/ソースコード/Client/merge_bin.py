Import("env")
import os

# ボード定義からチップ種別ごとのesptoolパラメータを取得
# (esp32 と esp32s3 でブートローダーオフセット/flash_mode/flash_freqが異なる)
board = env.BoardConfig()
mcu = board.get("build.mcu", "esp32")
flash_size = board.get("upload.flash_size", "4MB")
# pio標準アップロードは常に --flash-mode dio で書き込む(board.build.flash_modeが
# qio等でもコンパイル設定であり書き込みヘッダには使わない)。ここで build.flash_mode を
# 使うとQIO配線非対応の実機ではROMがフラッシュを読めずブートループする。
flash_mode = "dio"
f_flash_hz = int(str(board.get("build.f_flash", "40000000L")).rstrip("L"))
flash_freq = "{}m".format(f_flash_hz // 1000000)
bootloader_offset = "0x1000" if mcu == "esp32" else "0x0"

# 出力先
output_dir = os.path.join(env.subst("$BUILD_DIR"))
output_bin = os.path.join(output_dir, "merged-firmware.bin")

# 入力ファイル
bootloader = os.path.join(output_dir, "bootloader.bin")
partitions = os.path.join(output_dir, "partitions.bin")
app = os.path.join(output_dir, "firmware.bin")
# otadata (huge_app.csv の 0xe000): pio標準アップロードは自動で書き込むが
# merge_bin単体では明示しないと欠落し、ota_0を選択できず起動しなくなる
boot_app0 = os.path.join(
    env.PioPlatform().get_package_dir("framework-arduinoespressif32"),
    "tools", "partitions", "boot_app0.bin"
)

# 結合コマンド
env.AddPostAction(
    "buildprog",
    env.VerboseAction(
        '"$PYTHONEXE" -m esptool --chip {} merge_bin -o "{}" --flash_mode {} --flash_freq {} --flash_size {} '
        '{} "{}" 0x8000 "{}" 0xe000 "{}" 0x10000 "{}"'.format(
            mcu, output_bin, flash_mode, flash_freq, flash_size,
            bootloader_offset, bootloader, partitions, boot_app0, app
        ),
        "Merging firmware into merged-firmware.bin"
    )
)
