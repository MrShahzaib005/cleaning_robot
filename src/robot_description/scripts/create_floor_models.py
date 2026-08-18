#!/usr/bin/env python3
"""
GAZEBO FLOOR GENERATOR v4
Each 2x2m patch = UNIQUE Gazebo model with its OWN material
Uses <include> in world so each model loads separately
"""
import os, glob, shutil

HOME = os.path.expanduser("~")
GMOD = os.path.join(HOME, ".gazebo", "models")
DS = os.path.join(HOME, "cleaning_robot_ws/src/robot_description/ai_models/dataset")
WO = os.path.join(HOME, "cleaning_robot_ws/src/robot_description/worlds/new_world.world")

SZ = 2.0
XMN, XMX = -9.35, 9.35
YMN, YMX = -7.35, 7.35
TYPES = ["tiles", "wood", "carpet"]
N = 'name'

def get_imgs(folder):
    r = []
    for e in ["*.jpg","*.jpeg","*.png","*.JPG","*.JPEG","*.PNG"]:
        r.extend(glob.glob(os.path.join(folder, e)))
    r.sort()
    return r[:8]

def mk(mid, matid, img):
    d = os.path.join(GMOD, mid)
    if os.path.isdir(d): shutil.rmtree(d)
    td = os.path.join(d, "materials", "textures")
    sd = os.path.join(d, "materials", "scripts")
    os.makedirs(td, exist_ok=True)
    os.makedirs(sd, exist_ok=True)
    try:
        from PIL import Image
        Image.open(img).convert('RGB').resize((512,512)).save(os.path.join(td,"texture.png"))
    except:
        shutil.copy2(img, os.path.join(td,"texture.png"))

    mc = open(os.path.join(d, "model.config"), "w")
    mc.write('<?xml version="1.0"?>\n<model>\n')
    mc.write('  <%s>%s</%s>\n' % (N, mid, N))
    mc.write('  <version>1.0</version>\n  <sdf version="1.6">model.sdf</sdf>\n</model>\n')
    mc.close()

    ms = open(os.path.join(sd, mid + ".material"), "w")
    ms.write("material %s\n{\n  technique\n  {\n    pass\n    {\n      texture_unit\n      {\n        texture texture.png\n        scale 0.3 0.3\n      }\n    }\n  }\n}\n" % matid)
    ms.close()

    fr = 0.8
    sf = open(os.path.join(d, "model.sdf"), "w")
    sf.write('<?xml version="1.0"?>\n<sdf version="1.6">\n')
    sf.write('<model %s="%s"><static>true</static>\n' % (N, mid))
    sf.write('<link %s="link">\n' % N)
    sf.write('<collision %s="c"><geometry><box><size>%s %s 0.01</size></box></geometry>\n' % (N, SZ, SZ))
    sf.write('<surface><friction><ode><mu>%.1f</mu><mu2>%.1f</mu2></ode></friction></surface></collision>\n' % (fr,fr))
    sf.write('<visual %s="v"><geometry><box><size>%s %s 0.01</size></box></geometry>\n' % (N, SZ, SZ))
    sf.write('<material><script>\n')
    sf.write('<uri>model://%s/materials/scripts/</uri>\n' % mid)
    sf.write('<uri>model://%s/materials/textures/</uri>\n' % mid)
    sf.write('<%s>%s</%s>\n' % (N, matid, N))
    sf.write('</script></material></visual>\n')
    sf.write('</link></model></sdf>\n')
    sf.close()

if __name__ == "__main__":
    print("=" * 55)
    print("  FLOOR GENERATOR v4 — UNIQUE PER PATCH")
    print("=" * 55)

    for old in glob.glob(os.path.join(GMOD, "fp_*")):
        if os.path.isdir(old): shutil.rmtree(old)
    for old in glob.glob(os.path.join(GMOD, "floor_*")):
        if os.path.isdir(old): shutil.rmtree(old)
    os.makedirs(GMOD, exist_ok=True)

    imgs = {}
    for t in TYPES:
        imgs[t] = get_imgs(os.path.join(DS, t))
        print(f"  {t}: {len(imgs[t])} images")

    patches = []
    x = XMN + SZ/2; col = 0
    while x < XMX:
        y = YMN + SZ/2; row = 0
        while y < YMX:
            ti = (col + row) % 3
            ft = TYPES[ti]
            il = imgs[ft]
            if il:
                ii = (col*7 + row*3) % len(il)
                mid = "fp_%d_%d" % (col, row)
                matid = "Mat_%d_%d" % (col, row)
                mk(mid, matid, il[ii])
                patches.append((x, y, mid, ft))
            y += SZ; row += 1
        x += SZ; col += 1

    print(f"\n  {len(patches)} unique patch models created")

    # World file
    f = open(WO, "w")
    f.write('<?xml version="1.0" ?>\n<sdf version=\'1.7\'>\n')
    f.write('<world %s=\'default\'>\n' % N)
    f.write('<light %s=\'sun\' type=\'directional\'>\n' % N)
    f.write('  <cast_shadows>0</cast_shadows><pose>0 0 10 0 0 0</pose>\n')
    f.write('  <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.1 0.1 0.1 1</specular>\n')
    f.write('  <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>\n')
    f.write('  <direction>-0.5 0.1 -0.9</direction>\n')
    f.write('  <spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot>\n')
    f.write('</light>\n')
    f.write('<physics type=\'ode\'><max_step_size>0.002</max_step_size><real_time_factor>1</real_time_factor><real_time_update_rate>500</real_time_update_rate></physics>\n')
    f.write('<scene><ambient>0.5 0.5 0.5 1</ambient><background>0.7 0.7 0.7 1</background><shadows>false</shadows><grid>false</grid></scene>\n\n')

    # Patches via <include>
    for x, y, mid, ft in patches:
        f.write('<include><uri>model://%s</uri>' % mid)
        f.write('<%s>%s</%s>' % (N, mid, N))
        f.write('<pose>%.2f %.2f 0.005 0 0 0</pose>' % (x, y))
        f.write('<static>true</static></include>\n')

    # Walls
    f.write('\n<model %s=\'first__map\'><pose>0.055464 -0.011717 0 0 0 0</pose>\n' % N)
    for wn, sz, ps in [("Wall_4","13.75 0.15 2.5","-9.425 -0.55 0 0 0 -1.5708"),("Wall_5","19 0.15 2.5","0 -7.35 0 0 0 0"),("Wall_6","13.75 0.15 2.5","9.425 -0.55 0 0 0 1.5708"),("Wall_7","1.25 0.15 2.5","9.425 6.8 0 0 0 1.5708"),("Wall_8","19 0.15 2.5","0 7.35 0 0 0 3.14159"),("Wall_9","1.25 0.15 2.5","-9.425 6.8 0 0 0 -1.5708")]:
        f.write('<link %s=\'%s\'>' % (N, wn))
        f.write('<collision %s=\'%s_C\'><geometry><box><size>%s</size></box></geometry><pose>0 0 1.25 0 0 0</pose></collision>' % (N, wn, sz))
        f.write('<visual %s=\'%s_V\'><pose>0 0 1.25 0 0 0</pose><geometry><box><size>%s</size></box></geometry>' % (N, wn, sz))
        f.write('<material><script><uri>file://media/materials/scripts/gazebo.material</uri><%s>Gazebo/Bricks</%s></script><ambient>1 1 1 1</ambient></material></visual>' % (N, N))
        f.write('<pose>%s</pose></link>\n' % ps)
    f.write('<static>1</static></model>\n')

    # Jersey barriers
    for jn, jp in [("jb","2.65861 3.59614 0 0 0 0"),("jb0","2.46748 -4.29572 0 0 0 0"),("jb1","6.19212 -0.171686 0 0 0 -1.56399"),("jb2","-3.25694 3.39659 0 0 0 0"),("jb3","-4.22053 1.36421 0 0 0 -1.52479"),("jb4","-2.41649 -3.22462 0 0 0 -2.1045")]:
        f.write('<model %s=\'%s\'><static>1</static><link %s=\'link\'>' % (N, jn, N))
        f.write('<visual %s=\'v\'><geometry><mesh><uri>model://jersey_barrier/meshes/jersey_barrier.dae</uri></mesh></geometry></visual>' % N)
        f.write('<collision %s=\'u\'><pose>0 0 0.5715 0 0 0</pose><geometry><box><size>4.06542 0.3063 1.143</size></box></geometry></collision>' % N)
        f.write('<collision %s=\'b\'><pose>0 0 0.032258 0 0 0</pose><geometry><box><size>4.06542 0.8107 0.064516</size></box></geometry></collision>' % N)
        f.write('</link><pose>%s</pose></model>\n' % jp)

    f.write('<gravity>0 0 -9.8</gravity>\n')
    f.write('<magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>\n')
    f.write('<atmosphere type=\'adiabatic\'/>\n')
    f.write('<gui fullscreen=\'0\'><camera %s=\'user_camera\'>' % N)
    f.write('<pose>0 -22 20 0 0.75 1.5708</pose>')
    f.write('<view_controller>orbit</view_controller>')
    f.write('<projection_type>perspective</projection_type>')
    f.write('</camera></gui>\n')
    f.write('</world>\n</sdf>\n')
    f.close()

    print(f"  World: {WO}")
    print(f"\n  Run:")
    print(f"    cd ~/cleaning_robot_ws")
    print(f"    rm -rf build/ install/ log/")
    print(f"    colcon build --packages-select robot_description --symlink-install")
    print(f"    source install/setup.bash")
    print(f"    launch_robot")