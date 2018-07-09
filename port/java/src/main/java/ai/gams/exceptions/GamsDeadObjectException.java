package ai.gams.exceptions;

import ai.madara.exceptions.MadaraDeadObjectException;

public class GamsDeadObjectException extends Exception {

	/**
	 * version
	 */
	private static final long serialVersionUID = 1L;

    /**
     * Constructor
     */
	public GamsDeadObjectException() {
	}

    /**
     * Constructor with message
     * @param  message information to embed in the exception
     */
	public GamsDeadObjectException(String message) {
		super(message);
	}

    /**
     * Constructor with cause
     * @param  cause  source of the exception
     */
	public GamsDeadObjectException(Throwable cause) {
		super(cause);
	}

    /**
     * Constructor with cause and message
     * @param  message information to embed in the exception
     * @param  cause  source of the exception
     */
	public GamsDeadObjectException(String message, Throwable cause) {
		super(message, cause);
	}

    /**
     * Constructor with cause and message and suppression
     * @param  message information to embed in the exception
     * @param  cause  source of the exception
     * @param  enableSuppression  if true, suppress the throw
     * @param  writableStackTrace  if true, write stack trace
     */
	public GamsDeadObjectException(String message, Throwable cause, boolean enableSuppression,
			boolean writableStackTrace) {
		super(message, cause, enableSuppression, writableStackTrace);
	}

}
