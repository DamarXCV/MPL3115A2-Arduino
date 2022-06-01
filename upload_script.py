Import("env")

# please keep $SOURCE variable, it will be replaced with a path to firmware

# Generic
env.Replace(
    UPLOADER="sudo avrdude",
    UPLOADCMD="$UPLOADER $UPLOADERFLAGS $SOURCE"
)

# In-line command with arguments
env.Replace(
    UPLOADCMD="sudo avrdude -c linuxgpio -p atmega328p -v -U flash:w:$SOURCE:i"
)

# Python callback
def on_upload(source, target, env):
    print(source, target)
    firmware_path = str(source[0])
    # do something
    env.Execute("sudo avrdude -c linuxgpio -p atmega328p -v -U flash:w:" + firmware_path + ":i" )

env.Replace(UPLOADCMD=on_upload)