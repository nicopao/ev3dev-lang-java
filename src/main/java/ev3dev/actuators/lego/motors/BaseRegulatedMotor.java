package ev3dev.actuators.lego.motors;

import java.io.File;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.HashSet;
import java.util.List;
import java.util.Queue;
import java.util.Set;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import ev3dev.hardware.EV3DevMotorDevice;
import ev3dev.hardware.EV3DevPlatforms;
import ev3dev.sensors.Battery;
import ev3dev.utils.DataChannelRereader;
import ev3dev.utils.DataChannelRewriter;
import lejos.hardware.port.Port;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.RegulatedMotorListener;
import lejos.utility.Delay;

/**
 * Abstraction for a Regulated motors motors. The basic control methods are:
 * <code>forward, backward, reverseDirection, stop</code> and <code>flt</code>.
 * To set each motors's velocity, use {@link #setSpeed(int)
 * <code>setSpeed  </code> }. The maximum velocity of the motors is limited by
 * the battery voltage and load. With no load, the maximum degrees per second is
 * about 100 times the voltage (for the large EV3 motors). <br>
 * The velocity is regulated by comparing the tacho count with velocity times
 * elapsed time, and adjusting motors power to keep these closely matched.
 * Changes in velocity will be made at the rate specified via the
 * <code> setAcceleration(int acceleration)</code> method. The methods
 * <code>rotate(int angle) </code> and <code>rotateTo(int ange)</code> use the
 * tachometer to control the position at which the motors stops, usually within
 * 1 degree or 2.<br>
 * <br>
 * <b> Listeners.</b> An object implementing the {@link lejos.robotics
 * <code> RegulatedMotorListener </code> } interface may register with this
 * class. It will be informed each time the motors starts or stops. <br>
 * <b>Stall detection</b> If a stall is detected or if for some other reason the
 * speed regulation fails, the motors will stop, and <code>isStalled()</code >
 * returns <b>true</b>. <br>
 * Motors will hold their position when stopped. If this is not what you require
 * use the flt() method instead of stop().
 *
 * @author Roger Glassey
 * @author Andy Shaw
 * @author Juan Antonio Breña Moral
 */
public abstract class BaseRegulatedMotor extends EV3DevMotorDevice implements RegulatedMotor {

	private static final Logger log = LoggerFactory.getLogger(BaseRegulatedMotor.class);

	// Following should be set to the max SPEED (in deg/sec) of the motor when free
	// running and powered by 9V
	protected final int MAX_SPEED_AT_9V;

	private int speed = 360;
	protected int acceleration = 6000;

	private boolean regulationFlag = true;

	private final DataChannelRewriter modeWriter;
	private final DataChannelContainer channelContainer;

	private final List<RegulatedMotorListener> listenerList;

	// synch-related variables
	private BaseRegulatedMotor[] motorsSynchedWith;
	private boolean isBeingSynched;
	private BaseRegulatedMotor synch_responsible;
	private final int PING_START_LISTENERS = 1;
	private final int PING_STOP_LISTENERS = 0;
	private final int PING_NO_LISTENER = 0;
	private int listenersToPing;

	/**
	 * Constructor
	 *
	 * @param motorPort motor port
	 * @param moveP     moveP
	 * @param moveI     moveI
	 * @param moveD     moveD
	 * @param holdP     holdP
	 * @param holdI     holdI
	 * @param holdD     holdD
	 * @param offset    offset
	 * @param maxSpeed  maxSpeed
	 */
	public BaseRegulatedMotor(final Port motorPort, float moveP, float moveI, float moveD, float holdP, float holdI,
			float holdD, int offset, int maxSpeed) {

		List<RegulatedMotorListener> list = new ArrayList<>();
		listenerList = Collections.synchronizedList(list);

		if (log.isInfoEnabled()) {
			log.info("Configuring motor connected on Port: {}", motorPort.getName());
		}

		MAX_SPEED_AT_9V = maxSpeed;
		final EV3DevPlatforms ev3DevPlatforms = EV3DevPlatforms.getInstance();
		final String port = ev3DevPlatforms.getMotorPort(motorPort);

		if (log.isDebugEnabled()) {
			log.debug("Detecting motor on port: {}", port);
		}
		this.detect(LEGO_PORT, port);
		if (log.isDebugEnabled()) {
			log.debug("Setting port in mode: {}", TACHO_MOTOR);
		}
		// Initialise mode writer based on first detect and then write mode.
		modeWriter = new DataChannelRewriter(PATH_DEVICE + "/" + MODE);
		modeWriter.writeString(TACHO_MOTOR);

		Delay.msDelay(1000);
		this.detect(TACHO_MOTOR, port);

		Delay.msDelay(1000);

		channelContainer = new DataChannelContainer(PATH_DEVICE, this);

		// TODO Review to implement asynchronous solution
		channelContainer.writeCommand(RESET);
		// this.setStringAttribute(COMMAND, RESET);
		if (log.isDebugEnabled()) {
			log.debug("Motor ready to use on Port: {}", motorPort.getName());
		}

		this.setInSynch(false);
		this.listenersToPing = PING_NO_LISTENER;

	}

	/**
	 * Removes this motors from the motors regulation system. After this call the
	 * motors will be in float mode and will have stopped. Note calling any of the
	 * high level move operations (forward, rotate etc.), will automatically enable
	 * regulation.
	 *
	 * @return true iff regulation has been suspended.
	 */
	public boolean suspendRegulation() {
		this.regulationFlag = false;
		return this.regulationFlag;
	}

	/**
	 * @return the current tachometer count.
	 * @see lejos.robotics.RegulatedMotor#getTachoCount()
	 */
	public int getTachoCount() {
		if (this.isInSynch()) {
			log.warn("getTachoCount() cannot be executed within synch block. Returning the pre-synchronization value.");
		}
		return channelContainer.readTacho();
		// return getIntegerAttribute(POSITION);
	}

	/**
	 * Returns the current position that the motors regulator is trying to maintain.
	 * Normally this will be the actual position of the motors and will be the same
	 * as the value returned by getTachoCount(). However in some circumstances
	 * (activeMotors that are in the process of stalling, or activeMotors that have
	 * been forced out of position), the two values may differ. Note that if
	 * regulation has been suspended calling this method will restart it.
	 *
	 * @return the current position calculated by the regulator.
	 */
	public float getPosition() {
		return this.getTachoCount();
	}

	@Override
	public void forward() {
		this.setSpeedDirect(this.speed);
		run();
	}

	@Override
	public void backward() {
		this.setSpeedDirect(-this.speed);
		run();
	}

	private void run() {
		if (!this.regulationFlag) {
			this.channelContainer.writeCommand(RUN_DIRECT);
		} else {
			this.channelContainer.writeCommand(RUN_FOREVER);
		}

		this.updateListenersAfterStart();
	}

	/**
	 * Set the motors into float mode. This will stop the motors without braking and
	 * the position of the motors will not be maintained.
	 */
	@Override
	public void flt(boolean immediateReturn) {
		doStop(COAST, immediateReturn);
	}

	@Override
	public void flt() {
		flt(false);
	}

	@Override
	public void coast() {
		doStop(COAST, false);
	}

	/**
	 * Removes power from the motor and creates a passive electrical load. This is
	 * usually done by shorting the motor terminals together. This load will absorb
	 * the energy from the rotation of the motors and cause the motor to stop more
	 * quickly than coasting.
	 */
	public void brake() {
		doStop(BRAKE, false);
	}

	/**
	 * Causes the motor to actively try to hold the current position. If an external
	 * force tries to turn the motor, the motor will “push back” to maintain its
	 * position.
	 */
	@Override
	public void hold() {
		doStop(HOLD, false);
	}

	/**
	 * Causes motors to stop, pretty much instantaneously. In other words, the
	 * motors doesn't just stop; it will resist any further motion. Cancels any
	 * rotate() orders in progress
	 */
	public void stop() {
		stop(false);
	}

	@Override
	public void stop(boolean immediateReturn) {
		doStop(HOLD, immediateReturn);
	}

	/**
	 * Backend for all stop moves. This sets the stop action type and then triggers
	 * the stop action.
	 *
	 * @param mode            One of BRAKE, COAST and HOLD string constants.
	 * @param immediateReturn Whether the function should busy-wait until the motor
	 *                        stops reporting the 'running' state.
	 */
	private void doStop(String mode, boolean immediateReturn) {
		this.channelContainer.writeStopCommand(mode);
		this.channelContainer.writeCommand(STOP);

		// ignore immediateReturn=false if in synch
		if (!(immediateReturn || this.isInSynch())) {
			waitComplete();
		}

		this.updateListenersAfterStop();
	}

	/**
	 * This method returns <b>true </b> if the motors is attempting to rotate. The
	 * return value may not correspond to the actual motors movement.<br>
	 * For example, If the motors is stalled, isMoving() will return <b> true.
	 * </b><br>
	 * After flt() is called, this method will return <b>false</b> even though the
	 * motors axle may continue to rotate by inertia. If the motors is stalled,
	 * isMoving() will return <b> true. </b> . A stall can be detected by calling
	 * {@link #isStalled()};
	 *
	 * @return true iff the motors is attempting to rotate.<br>
	 */
	@Override
	public boolean isMoving() {
		if (this.isInSynch()) {
			log.warn("isMoving() cannot be executed within synch block. Returning the pre-synchronization value.");
		}
		return (this.channelContainer.readState().contains(STATE_RUNNING));
		// return (this.getStringAttribute(STATE).contains(STATE_RUNNING));
	}

	/**
	 * Sets desired motors speed , in degrees per second; The maximum reliably
	 * sustainable velocity is 100 x battery voltage under moderate load, such as a
	 * direct drive robot on the level.
	 *
	 * @param speed value in degrees/sec
	 */
	public void setSpeed(int speed) {
		setSpeedDirect(speed);
		run();
		this.speed = speed;
	}

	private void setSpeedDirect(int speed) {
		if (!this.regulationFlag) {
			this.channelContainer.writeDutyCycle(speed);
		} else {
			this.channelContainer.writeSpeed(speed);
		}
	}

	/**
	 * Reset the tachometer associated with this motors. Note calling this method
	 * will cause any current move operation to be halted.
	 */
	public void resetTachoCount() {
		this.channelContainer.writeCommand(RESET);
		this.regulationFlag = true;
	}

	/**
	 * Rotate by the request number of degrees.
	 *
	 * @param angle           number of degrees to rotate relative to the current
	 *                        position
	 * @param immediateReturn if true do not wait for the move to complete Rotate by
	 *                        the requested number of degrees. Wait for the move to
	 *                        complete.
	 */
	public void rotate(int angle, boolean immediateReturn) {
		this.setSpeedDirect(this.speed);
		this.channelContainer.writePositionSP(angle);
		this.channelContainer.writeCommand(RUN_TO_REL_POS);

		// don't block if in synch
		if (!(immediateReturn || this.isInSynch())) {
			while (this.isMoving()) {
				// do stuff or do nothing
				// possibly sleep for some short interval to not block
			}
		}

		this.updateListenersAfterStart();
	}

	/**
	 * Rotate by the requested number of degrees. Do not wait for the move to
	 * complete by default.
	 *
	 * @param angle angle
	 */
	public void rotate(int angle) {
		rotate(angle, false);
	}

	/**
	 * Rotate to a specific angle
	 *
	 * @param limitAngle      angle
	 * @param immediateReturn If the method behave in an asynchronous way
	 */
	public void rotateTo(int limitAngle, boolean immediateReturn) {
		this.setSpeedDirect(this.speed);

		this.channelContainer.writePositionSP(limitAngle);
		this.channelContainer.writeCommand(RUN_TO_ABS_POS);

		// don't block if in synch
		if (!(immediateReturn || this.isInSynch())) {
			while (this.isMoving()) {
				// do stuff or do nothing
				// possibly sleep for some short interval to not block
			}
		}

		this.updateListenersAfterStart();
	}

	/**
	 * Rotate to the target angle. Do not return until the move is complete.
	 *
	 * @param limitAngle Angle to rotate to.
	 */
	public void rotateTo(int limitAngle) {
		rotateTo(limitAngle, false);
	}

	/**
	 * Return the current target speed.
	 *
	 * @return the current target speed.
	 */
	public int getSpeed() {
		if (this.isInSynch()) {
			log.warn("getSpeed() cannot be executed within synch block. Returning the pre-synchronization value.");
		}

		if (!this.regulationFlag) {
			return this.channelContainer.readDutyCycle();
			// return this.getIntegerAttribute(DUTY_CYCLE);
		} else {
			return this.channelContainer.readSpeed();
			// return this.getIntegerAttribute(SPEED);
		}

	}

	/**
	 * Return true if the motors is currently stalled.
	 *
	 * @return true if the motors is stalled, else false
	 */
	public boolean isStalled() {
		if (this.isInSynch()) {
			log.warn("isStalled() cannot be executed within synch block. Returning the pre-synchronization value.");
		}
		return (this.channelContainer.readState().contains(STATE_STALLED));
		// return (this.getStringAttribute(STATE).contains(STATE_STALLED));
	}

	/**
	 * Return the current velocity.
	 *
	 * @return current velocity in degrees/s
	 */
	public int getRotationSpeed() {
		return 0;// Math.round(reg.getCurrentVelocity());
	}

	@Override
	public void addListener(RegulatedMotorListener regulatedMotorListener) {
		listenerList.add(regulatedMotorListener);
	}

	@Override
	public RegulatedMotorListener removeListener() {
		if (listenerList.size() > 0) {
			listenerList.remove(listenerList.size() - 1);
		}
		return null;
	}

	@Override
	public void waitComplete() {
		// TODO Review the side effect with multiple motors
		while (this.isMoving()) {
			Delay.msDelay(1);
		}
	}

	@Override
	public float getMaxSpeed() {
		// It is generally assumed, that the maximum accurate speed of an EV3 Motor is
		// 100 degree/second * Voltage. We generalise this to other LEGO motors by
		// returning a value
		// that is based on 90% of the maximum free running speed of the motor.
		// TODO: Should this be using the Brick interface?
		// TODO: If we ever allow the regulator class be remote, then we will need to
		// ensure we
		// get the voltage of the remote brick not the local one.
		// return LocalEV3.ev3.getPower().getVoltage() * MAX_SPEED_AT_9V/9.0f * 0.9f;
		return Battery.getInstance().getVoltage() * MAX_SPEED_AT_9V / 9.0f * 0.9f;
	}

	// below two methods wrap the listeners update mechanism
	// if under synchronisation, it will postpone the execution at the end of
	// end-synch
	// it clearly doesn't make sense to notify listeners if both start and stop
	// actions happen
	// so it will notify only about the last one
	private void updateListenersAfterStop() {
		if (this.isInSynch()) {
			this.listenersToPing = PING_STOP_LISTENERS;
		} else {
			for (RegulatedMotorListener listener : listenerList) {
				listener.rotationStopped(this, this.getTachoCount(), this.isStalled(), System.currentTimeMillis());
			}
		}
	}

	private void updateListenersAfterStart() {
		if (this.isInSynch()) {
			this.listenersToPing = PING_START_LISTENERS;
		} else {
			for (RegulatedMotorListener listener : listenerList) {
				listener.rotationStarted(this, this.getTachoCount(), this.isStalled(), System.currentTimeMillis());
			}
		}
	}

	@Override
	/**
	 * sets the acceleration rate of this motor in degrees/sec/sec <br>
	 * The default value is 6000; Smaller values will make speeding up. or stopping
	 * at the end of a rotate() task, smoother;
	 * 
	 * @param acceleration
	 */
	public void setAcceleration(int acceleration) {
		this.acceleration = Math.abs(acceleration);

		log.warn("Not executed internally the method: setAcceleration");
		// reg.adjustAcceleration(this.acceleration);
	}

	@Override
	public void synchronizeWith(RegulatedMotor[] regulatedMotors) {
		// we need the BaseRegulatedMotor type to access the full range of synch methods
		if (this.isInSynch()) {
			log.warn("Can't change synchronised-with motors while in a synchronisation block");
		} else {
			this.motorsSynchedWith = (BaseRegulatedMotor[]) regulatedMotors;
		}
	}

	@Override
	public void startSynchronization() {
		// this is the motor responsible
		this.setSynchResponsible(this);
		this.setInSynch(true);
		
		// remove duplicated motors from synched list
		// use a set for this purpose
		Set<BaseRegulatedMotor> synched = new HashSet<BaseRegulatedMotor>();
		Collections.addAll(synched,this.motorsSynchedWith);
		this.motorsSynchedWith = (BaseRegulatedMotor[]) synched.toArray();

		for (BaseRegulatedMotor otherMotor : this.motorsSynchedWith) {
			otherMotor.setSynchResponsible(this);
			otherMotor.setInSynch(true);
		}
	}

	@Override
	public void endSynchronization() {

		if (this.getSynchResponsible() != this)
			log.warn("ignoring endSynchronization: it was invoked on wrong object");
		else if (!this.isInSynch())
			log.warn("ignoring endSynchronization: startSynchronisation not found");

		else {
			// begin scheduling
			List<BaseRegulatedMotor> motorlist = new ArrayList<BaseRegulatedMotor>();
			motorlist.add(this);
	        Collections.addAll(motorlist, this.motorsSynchedWith);
			// this arrays keeps the number of actions left to schedule for each synchedwithmotor+this one
			// init array
			int[] actionsLeft = new int[motorlist.size()];
			int totalActions = 0;
			for(int i =0; i< motorlist.size(); i++) {
				BaseRegulatedMotor m = motorlist.get(i);
				int numActions = m.channelContainer.getNumOfDispatchedAtomicActions();
				actionsLeft[i]=numActions;
				totalActions+=numActions;
			}

			// array to store the scheduling
			BaseRegulatedMotor[] schedule = new BaseRegulatedMotor[totalActions];
			for (int i =0; i< schedule.length; i++) {
				// for efficiency, schedule by default the first motor 
				schedule[i]=motorlist.get(0);
				actionsLeft[0]=actionsLeft[0]-1;
				double coin = Math.random()*(totalActions-i);
				double cdf = 0;
				// don't need to check the first
				for (int j=1; j<actionsLeft.length; j++) {
					cdf+=actionsLeft[j];
					if(coin<cdf) {
						schedule[i]=motorlist.get(j);
						actionsLeft[0]=actionsLeft[0]+1;//undo the default case
						actionsLeft[j]=actionsLeft[j]-1;
						break;
					}
				}
			}
			

//    	write schedule in private method, and test it just with simple objects 
//		execute step by step
//    	TODO "undo" the startSynchronization commmands
//		empty list of dispatched actions (it should be empty, check)
//		for each writer, check that state is 0 (if not, it's inconsistent, so reset the buffer or something) 
//    	run listeners
			// execute this.listenersToPing = PING_NO_LISTENER; for all motors
		}
	}

	/**
	 * @return true if the motor is involved in a synchronisation block, false
	 *         otherwise
	 */
	private boolean isInSynch() {
		return isBeingSynched;
	}

	/**
	 * flag motor as one involved (or not) in synchronisation
	 * 
	 * @param inSynch whether or not motor in synch block
	 */
	private void setInSynch(boolean inSynch) {
		this.isBeingSynched = inSynch;
	}

	/**
	 * @return the motor responsible to handle synchronisation
	 */
	private BaseRegulatedMotor getSynchResponsible() {
		return this.synch_responsible;
	}

	/**
	 * @param synch_responsible the synch_responsible to set
	 */
	private void setSynchResponsible(BaseRegulatedMotor responsible) {
		this.synch_responsible = responsible;
	}

	private class DataChannelContainer {

		// DATACHANNEL REWRITERS
		private final DataChannelRewriter speedWriter;
		private final DataChannelRewriter commandWriter;
		private final DataChannelRewriter dutyCycleWriter;
		private final DataChannelRewriter stopCommandWriter;
		private final DataChannelRewriter positionSPWriter;

		// DATACHANEL REREADERS
		private final DataChannelRereader stateReader;
		private final DataChannelRereader tachoReader;
		private final DataChannelRereader speedReader;
		private final DataChannelRereader dutyCycleReader;

		private final File path;

		// queue to store list of actions dispatched during a synchronisation
		private class Action {
			private DataChannelRewriter writer;
			private String arg;

			public Action(DataChannelRewriter writer, String arg) {
				this.writer = writer;
				this.arg = arg;
			}

			public DataChannelRewriter getWriter() {
				return this.writer;
			}

			public String getArg() {
				return this.arg;
			}
			
			public String toString() {
				return "WRITE "+this.arg+" into "+this.writer.getPath().getFileName();
			}
		}

		private Deque<Action> dispatchedActions;

		private BaseRegulatedMotor owner;

		public DataChannelContainer(File path, BaseRegulatedMotor owner) {
			this.path = path;

			// Initialising writers.
			speedWriter = new DataChannelRewriter(path + "/" + SPEED);
			commandWriter = new DataChannelRewriter(path + "/" + COMMAND);
			stopCommandWriter = new DataChannelRewriter(path + "/" + STOP_COMMAND);
			dutyCycleWriter = new DataChannelRewriter(path + "/" + DUTY_CYCLE);
			positionSPWriter = new DataChannelRewriter(path + "/" + POSITION_SP);

			// Initialising readers.
			stateReader = new DataChannelRereader(path + "/" + STATE);
			tachoReader = new DataChannelRereader(path + "/" + POSITION);
			speedReader = new DataChannelRereader(path + "/" + SPEED);
			dutyCycleReader = new DataChannelRereader(path + "/" + DUTY_CYCLE);

			this.owner = owner;
			this.dispatchedActions = new ArrayDeque<Action>();
		}

		public int getNumOfDispatchedAtomicActions() {
			int count = 0;
			for (Action a : this.dispatchedActions)
				count += a.getWriter().getNumOfStates();
			return count;
		}

//		executes next dispatched action (only if in synch)
//		returns whether or not the queue is empty
		public boolean executeNext() {
			if (this.owner.isInSynch()) {
				// get first action in queue (don't remove it unless it's finished)
				Action a = this.dispatchedActions.getFirst();
				// if it returns false, action a has finished executing
				while (!a.getWriter().executeNext(a.getArg()) && !this.dispatchedActions.isEmpty()) {
					// removes the action and updates next
					this.dispatchedActions.poll();
					if(!this.dispatchedActions.isEmpty())
						a = this.dispatchedActions.getFirst();
				}
			}
			return this.dispatchedActions.isEmpty();
		}

		public void writeSpeed(int speed) {
			if (!this.owner.isInSynch())
				speedWriter.writeInt(speed);
			else
				this.dispatchedActions.add(new Action(speedWriter, speed + ""));
		}

		public void writeDutyCycle(int dutyCycle) {
			if (!this.owner.isInSynch())
				dutyCycleWriter.writeInt(dutyCycle);
			else
				this.dispatchedActions.add(new Action(dutyCycleWriter, dutyCycle + ""));
		}

		public void writeCommand(String command) {
			if (!this.owner.isInSynch())
				commandWriter.writeString(command);
			else
				this.dispatchedActions.add(new Action(commandWriter, command));
		}

		public void writeStopCommand(String stopCommand) {
			if (!this.owner.isInSynch())
				stopCommandWriter.writeString(stopCommand);
			else
				this.dispatchedActions.add(new Action(stopCommandWriter, stopCommand));
		}

		public void writePositionSP(int posSP) {
			if (!this.owner.isInSynch())
				positionSPWriter.writeInt(posSP);
			else
				this.dispatchedActions.add(new Action(positionSPWriter, posSP + ""));
		}

		public String readState() {
			return stateReader.readString();
		}

		public int readTacho() {
			return tachoReader.readInt();
		}

		public int readSpeed() {
			return speedReader.readInt();
		}

		public int readDutyCycle() {
			return dutyCycleReader.readInt();
		}

	}

}
