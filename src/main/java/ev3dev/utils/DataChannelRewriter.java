package ev3dev.utils;

import java.io.Closeable;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

/**
 * Writer of streams that can rewrite the same channel for structured data of
 * known length. The focus of this class is on performance.
 *
 * @author David Walend
 */
public class DataChannelRewriter implements Closeable {

	private final Path path;
	private final ByteBuffer byteBuffer;
	private final FileChannel channel;

	// variables for step-by-step execution
	private int state;

	private final int NUM_OF_STATES = 7;

	/**
	 * Create a DataChannelRewriter for path with a bufferLength byte buffer
	 *
	 * @param path         path to the file to reread
	 * @param bufferLength length of the buffer to hold the structure
	 */
	public DataChannelRewriter(Path path, int bufferLength) {
		this.path = path;
		this.byteBuffer = ByteBuffer.allocate(bufferLength);
		try {
			this.channel = FileChannel.open(path, StandardOpenOption.WRITE);
		} catch (IOException e) {
			throw new RuntimeException("While opening " + path, e);
		}
		this.state = 0;
	}

	public int getState() {
		return state;
	}

	public void resetState(boolean warn) {
		if (warn && this.state != 0)
			System.out.println("Resetting non-initial state of writer" + this.path.getFileName());
		this.state = 0;
	}

	/**
	 * Create a DataChannelRewriter for pathString with the default 32-byte buffer.
	 *
	 * @param pathString Path to the file to reread
	 */
	public DataChannelRewriter(String pathString) {
		this(Paths.get(pathString), 32);
	}

	// returns false if all steps are executed
	// TODO method comments
	public boolean executeNext(String string) {
		if (this.state < 0 || this.state > 6) {
			return false;
		} else {
			this.steppedWrite(string);
			return true;
		}

	}

	private synchronized void steppedWrite(String string) {
		try {
			switch (this.state) {
			case 0:
				byteBuffer.clear();
			case 1:
				byteBuffer.put(string.getBytes(StandardCharsets.UTF_8));
			case 2:
				byteBuffer.put(((byte) '\n'));
			case 3:
				byteBuffer.flip();
			case 4:
				channel.truncate(0);
			case 5:
				channel.write(byteBuffer, 0);
			case 6:
				channel.force(false);
			default:
				this.state = -1;// it will become 0 after return
			}
			this.state++;
		} catch (IOException e) {
			throw new RuntimeException("Problem writing path: " + path, e);
		}
	}

	/**
	 * @param string to write. A new line character
	 */
	public synchronized void writeString(String string) {
		if (this.state != 0) {
			System.err.println("Can't write " + string + " on " + this.path.getFileName()
					+ ". Data channel is currently inside non-atomic write transaction");
		} else {
			try {
				byteBuffer.clear();
				byteBuffer.put(string.getBytes(StandardCharsets.UTF_8));
				byteBuffer.put(((byte) '\n'));
				byteBuffer.flip();
				channel.truncate(0);
//            long t = System.currentTimeMillis();
				channel.write(byteBuffer, 0);
				channel.force(false);
			} catch (IOException e) {
				throw new RuntimeException("Problem writing path: " + path, e);
			}
		}
	}

	public synchronized void writeInt(int intToWrite) {
		writeString(Integer.toString(intToWrite));
	}

	public Path getPath() {
		return path;
	}

	@Override
	public synchronized void close() throws IOException {
		channel.close();
	}

	public int getNumOfStates() {
		return NUM_OF_STATES;
	}
}
