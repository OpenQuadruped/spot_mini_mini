import pybullet as p
import pybullet_data as pd
import math
import time

textureId = -1

useProgrammatic = 0
useTerrainFromPNG = 1
useDeepLocoCSV = 2
updateHeightfield = False

heightfieldSource = useProgrammatic
import random
random.seed(10)


class HeightField():
    def __init__(self):
        self.hf_id = 0

    def _generate_field(self, env, heightPerturbationRange=0.08):

        env.pybullet_client.setAdditionalSearchPath(pd.getDataPath())

        env.pybullet_client.configureDebugVisualizer(
            env.pybullet_client.COV_ENABLE_RENDERING, 0)
        heightPerturbationRange = heightPerturbationRange
        if heightfieldSource == useProgrammatic:
            numHeightfieldRows = 256
            numHeightfieldColumns = 256
            heightfieldData = [0] * numHeightfieldRows * numHeightfieldColumns
            for j in range(int(numHeightfieldColumns / 2)):
                for i in range(int(numHeightfieldRows / 2)):
                    height = random.uniform(0, heightPerturbationRange)
                    heightfieldData[2 * i +
                                    2 * j * numHeightfieldRows] = height
                    heightfieldData[2 * i + 1 +
                                    2 * j * numHeightfieldRows] = height
                    heightfieldData[2 * i +
                                    (2 * j + 1) * numHeightfieldRows] = height
                    heightfieldData[2 * i + 1 +
                                    (2 * j + 1) * numHeightfieldRows] = height

            terrainShape = env.pybullet_client.createCollisionShape(
                shapeType=env.pybullet_client.GEOM_HEIGHTFIELD,
                meshScale=[.05, .05, 1],
                heightfieldTextureScaling=(numHeightfieldRows - 1) / 2,
                heightfieldData=heightfieldData,
                numHeightfieldRows=numHeightfieldRows,
                numHeightfieldColumns=numHeightfieldColumns)
            terrain = env.pybullet_client.createMultiBody(
                0, terrainShape)
            env.pybullet_client.resetBasePositionAndOrientation(
                terrain, [0, 0, 0], [0, 0, 0, 1])

        if heightfieldSource == useDeepLocoCSV:
            terrainShape = env.pybullet_client.createCollisionShape(
                shapeType=env.pybullet_client.GEOM_HEIGHTFIELD,
                meshScale=[.5, .5, 2.5],
                fileName="heightmaps/ground0.txt",
                heightfieldTextureScaling=128)
            terrain = env.pybullet_client.createMultiBody(
                0, terrainShape)
            env.pybullet_client.resetBasePositionAndOrientation(
                terrain, [0, 0, 0], [0, 0, 0, 1])

        if heightfieldSource == useTerrainFromPNG:
            terrainShape = env.pybullet_client.createCollisionShape(
                shapeType=env.pybullet_client.GEOM_HEIGHTFIELD,
                meshScale=[.05, .05, 5],
                fileName="heightmaps/wm_height_out.png")
            textureId = env.pybullet_client.loadTexture(
                "heightmaps/gimp_overlay_out.png")
            terrain = env.pybullet_client.createMultiBody(
                0, terrainShape)
            env.pybullet_client.changeVisualShape(terrain,
                                                  -1,
                                                  textureUniqueId=textureId)

        self.hf_id = terrainShape
        print("TERRAIN SHAPE: {}".format(terrainShape))

        env.pybullet_client.changeVisualShape(terrain,
                                              -1,
                                              rgbaColor=[1, 1, 1, 1])

        env.pybullet_client.configureDebugVisualizer(
            env.pybullet_client.COV_ENABLE_RENDERING, 1)
