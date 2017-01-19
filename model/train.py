from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import TensorBoard
import model as traffic_light_model
batch_size = 64
nb_epoch = 50

model = traffic_light_model.get_model()
datagen = ImageDataGenerator(width_shift_range=.3, height_shift_range=.3, shear_range=0.05, fill_mode='nearest', rescale=1. / 255, horizontal_flip=True)
image_data_gen = datagen.flow_from_directory('images', target_size=traffic_light_model.IMAGE_DIMENSIONS, classes=['green', 'red', 'unknown'],
                                             batch_size=batch_size)

tensorboard_callback = TensorBoard(log_dir='/tmp/tf-lights', histogram_freq=1)

model.fit_generator(image_data_gen, nb_epoch=nb_epoch, samples_per_epoch=image_data_gen.nb_sample, callbacks=[tensorboard_callback])

model.save('light_classifier_model.h5')
