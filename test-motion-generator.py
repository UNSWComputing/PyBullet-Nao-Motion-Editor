# MotionHandler Test

from MotionHandle import MotionHandle as MH

test_motion = MH()

# Motion generation test
# test_motion.addKeyframe(1000, [i for i in range(25)], description="test motion step 1")
# test_motion.addKeyframe(10000, [i*2 for i in range(25)], description="test motion step 2")
# test_motion.addKeyframe(3400, [i*3 for i in range(25)], description="test motion step 3")

# test_motion.generatePosFile("test-motion")

# Reading .pos files
test_motion.readPosFile("sit.pos")