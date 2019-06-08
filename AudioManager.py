import mingus.midi.fluidsynth as fs
from mingus.containers.bar import Bar
from pygame import mixer

class AudioManager(object):
	def __init__(self):

		self._notes = []
		self._bar = None
		self._durs = []
		self._index = 0
		self._key = ""
		self._bpm = 120
		self._dynamics = {}

		self.paused = False

		self._bar = Bar()

		fs.init("sound.sf2", "oss")
		mixer.init()

	def getCurrentNote(self):
		return self._notes[self._index]

	def setInstrument(self, num):
		fs.set_instrument(1, num)

	def playNextNoteLong(self):
		"""Plays the next note in the song and advances the index"""
		tempBar = Bar()
		tempBar.set_meter((1, 1))
		tempBar.key = self._key
		tempBar.place_notes(self._notes[self._index], 1)

		fs.play_Bar(tempBar, 120)

	def playNote(self, note):
		"""Plays the next note in the song and advances the index"""
		tempBar = Bar()
		tempBar.place_notes(note, 1)

		fs.play_Bar(tempBar, 1, 200)

	def playChord(self, note1, note2):
		"""Plays the next note in the song and advances the index"""
		tempBar = Bar()
		tempBar.place_notes([note1, note2], 1)

		fs.play_Bar(tempBar, 1, 200)

	def playManyNotes(self, notes):
		"""Plays the next note in the song and advances the index"""
		tempBar = Bar()
		tempBar.set_meter((len(notes) * 4, 4))
		for n in notes:
			tempBar.place_notes(n, 1)

		fs.play_Bar(tempBar, 1, 200)

	def playNextNote(self):
		"""Plays the next note in the song and advances the index"""
		tempBar = Bar()
		beats = 1.0 / (1.0 / self._beat) / (1.0 / self._durs[self._index])
		tempBar.set_meter((beats, self._beat))
		tempBar.key = self._key
		tempBar.place_notes(self._notes[self._index], self._durs[self._index])

		print tempBar

		fs.play_Bar(tempBar, self._bpm)

		return self._index

	def loadMP3(self, filename):
		mixer.music.load(filename)

	def playMP3(self):
		self.paused = False
		mixer.music.play()

	def pauseMP3(self):
		if not self.paused:
			mixer.music.pause()
			self.paused = True

	def unpauseMP3(self):
		if self.paused:
			mixer.music.unpause()
			self.paused = False

	def stopMP3(self):
		mixer.music.stop()

	def advance(self):
		self._index += 1

	def reset(self):
		self._index = 0

	def songComplete(self):
		return self._index >= len(self._durs)

	def changeNote(self, notes):
		self._notes[self._index] = notes

		return self._index

	def loadSong(self, fn, isTwoHand):
		"""Loads in a song from the specified file"""
		header = dict()
		length = 0.0

		self._notes = []
		self._bar = None
		self._durs = []

		with open(fn) as file:
			i = 0
			for line in file:
				if line[0] == "#" or line[0].strip() == "":		# Skips blank lines and comment lines
					continue
				if i < 3:										# The first three lines are loaded into the header dictionary
					vals = line.split('=')
					header[vals[0]] = vals[1].strip()
					i += 1
				else:
					vals = line.split(',')
					if line[0] == 'r':							# Indicates a rest value
						self._notes.append(None)
						dur = vals[1].strip()
					else:
						if isTwoHand:							# If isTwoHand is true, then a two note chord is looked for
							self._notes.append([vals[0].strip(), vals[1].strip()])
							dur = vals[2].strip()
						else:
							self._notes.append(vals[0].strip())
							dur = vals[1].strip()
					if dur[len(dur) - 1] == '.':				# Looks for dotted notes
						dur = value.dots(int(dur[:-1]))
					else:
						dur = int(dur)
					self._durs.append(dur)
					length += 1.0 / dur

		self._key = header["key"]
		self._bpm = int(header["bpm"])
		self._beat = int(header["beat"])
		length *= self._bpm
		self._bar = Bar()
		self._bar.set_meter((length, int(header["beat"])))

	def stop(self):
		"""Stops all music"""
		fs.stop_everything()
