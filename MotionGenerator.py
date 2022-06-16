# Pos file Generator
# import sys

class MotionGenerator:
    def __init__(self):
        self.keyframes = []
        self.defaultStiffness = 1.0
        self.header = "  HY    HP    LSP   LSR   LEY   LER   LWY   LHYP  LHR   LHP   LKP   LAP   LAR   RHR   RHP   RKP   RAP   RAR   RSP   RSR   REY   RER   RWY   LH    RH    DUR"

    def addKeyframe(self, duration, jointvals, stiffnessvals=[], description=""):
        new_keyframe = {
                        "description":   description, 
                        "joint_vals":     jointvals, 
                        "stiffness_vals": stiffnessvals,
                        "duration":      duration
                        }
        self.keyframes.append(new_keyframe)

    def generatePosFile(self, filename):
        with open(filename+".pos", 'w') as posFile:
            posFile.write(f'{self.header}\n\n')          
            for keyframe in self.keyframes:
                posFile.write(f'{keyframe["description"]}\n')
                
                # If stiffness is not specified, set them to the default value of 1.0
                if len(keyframe["stiffness_vals"]) == 0:
                    keyframe["stiffness_vals"] = [self.defaultStiffness] * 25

                posFile.write("$ ")
                for stiffnessval in keyframe["stiffness_vals"]:
                    posFile.write(f'{stiffnessval:<6}')

                posFile.write("\n! ")
                for jointval in keyframe["joint_vals"]:
                    posFile.write(f'{jointval:<6}')
                
                posFile.write(f'{keyframe["duration"]}\n\n')
