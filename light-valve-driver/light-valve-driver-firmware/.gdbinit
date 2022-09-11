# Don't ask to confirm quit
define hook-quit
    set confirm off
end

# Don't ask to confirm del all bp
define hook-delete
    set confirm off
end

target extended-remote :3333
load
b main
c
