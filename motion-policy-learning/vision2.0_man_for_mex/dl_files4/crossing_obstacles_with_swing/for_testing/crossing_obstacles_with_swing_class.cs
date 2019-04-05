/*
* MATLAB Compiler: 6.2 (R2016a)
* Date: Mon May 14 17:37:37 2018
* Arguments: "-B" "macro_default" "-W"
* "dotnet:crossing_obstacles_with_swing,crossing_obstacles_with_swing_class,0.0,private"
* "-T" "link:lib" "-d"
* "C:\Graduation_Project\One_Step_V8_improving\vision2.0_man_for_mex\dl_files4\crossing_ob
* stacles_with_swing\for_testing" "-v"
* "class{crossing_obstacles_with_swing_class:C:\Graduation_Project\One_Step_V8_improving\v
* ision2.0_man_for_mex\crossing_obstacles_with_swing.m}" 
*/
using System;
using System.Reflection;
using System.IO;
using MathWorks.MATLAB.NET.Arrays;
using MathWorks.MATLAB.NET.Utility;

#if SHARED
[assembly: System.Reflection.AssemblyKeyFile(@"")]
#endif

namespace crossing_obstacles_with_swing
{

  /// <summary>
  /// The crossing_obstacles_with_swing_class class provides a CLS compliant, MWArray
  /// interface to the MATLAB functions contained in the files:
  /// <newpara></newpara>
  /// C:\Graduation_Project\One_Step_V8_improving\vision2.0_man_for_mex\crossing_obstacles
  /// _with_swing.m
  /// </summary>
  /// <remarks>
  /// @Version 0.0
  /// </remarks>
  public class crossing_obstacles_with_swing_class : IDisposable
  {
    #region Constructors

    /// <summary internal= "true">
    /// The static constructor instantiates and initializes the MATLAB Runtime instance.
    /// </summary>
    static crossing_obstacles_with_swing_class()
    {
      if (MWMCR.MCRAppInitialized)
      {
        try
        {
          Assembly assembly= Assembly.GetExecutingAssembly();

          string ctfFilePath= assembly.Location;

          int lastDelimiter= ctfFilePath.LastIndexOf(@"\");

          ctfFilePath= ctfFilePath.Remove(lastDelimiter, (ctfFilePath.Length - lastDelimiter));

          string ctfFileName = "crossing_obstacles_with_swing.ctf";

          Stream embeddedCtfStream = null;

          String[] resourceStrings = assembly.GetManifestResourceNames();

          foreach (String name in resourceStrings)
          {
            if (name.Contains(ctfFileName))
            {
              embeddedCtfStream = assembly.GetManifestResourceStream(name);
              break;
            }
          }
          mcr= new MWMCR("",
                         ctfFilePath, embeddedCtfStream, true);
        }
        catch(Exception ex)
        {
          ex_ = new Exception("MWArray assembly failed to be initialized", ex);
        }
      }
      else
      {
        ex_ = new ApplicationException("MWArray assembly could not be initialized");
      }
    }


    /// <summary>
    /// Constructs a new instance of the crossing_obstacles_with_swing_class class.
    /// </summary>
    public crossing_obstacles_with_swing_class()
    {
      if(ex_ != null)
      {
        throw ex_;
      }
    }


    #endregion Constructors

    #region Finalize

    /// <summary internal= "true">
    /// Class destructor called by the CLR garbage collector.
    /// </summary>
    ~crossing_obstacles_with_swing_class()
    {
      Dispose(false);
    }


    /// <summary>
    /// Frees the native resources associated with this object
    /// </summary>
    public void Dispose()
    {
      Dispose(true);

      GC.SuppressFinalize(this);
    }


    /// <summary internal= "true">
    /// Internal dispose function
    /// </summary>
    protected virtual void Dispose(bool disposing)
    {
      if (!disposed)
      {
        disposed= true;

        if (disposing)
        {
          // Free managed resources;
        }

        // Free native resources
      }
    }


    #endregion Finalize

    #region Methods

    /// <summary>
    /// Provides a single output, 0-input MWArrayinterface to the
    /// crossing_obstacles_with_swing MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// This function outputs a time profile of joint angles, taking the trajectory
    /// requirements as well as environment settings as input.
    /// Output: 'the_tot' is a time profile of joint angles, 'f_tot' is a time profile of
    /// ducted fan's thrust 
    /// Input: traj_setting(struct), motion_req(struct), envionment_setting
    /// </remarks>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray crossing_obstacles_with_swing()
    {
      return mcr.EvaluateFunction("crossing_obstacles_with_swing", new MWArray[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input MWArrayinterface to the
    /// crossing_obstacles_with_swing MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// This function outputs a time profile of joint angles, taking the trajectory
    /// requirements as well as environment settings as input.
    /// Output: 'the_tot' is a time profile of joint angles, 'f_tot' is a time profile of
    /// ducted fan's thrust 
    /// Input: traj_setting(struct), motion_req(struct), envionment_setting
    /// </remarks>
    /// <param name="swing_radius">Input argument #1</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray crossing_obstacles_with_swing(MWArray swing_radius)
    {
      return mcr.EvaluateFunction("crossing_obstacles_with_swing", swing_radius);
    }


    /// <summary>
    /// Provides a single output, 2-input MWArrayinterface to the
    /// crossing_obstacles_with_swing MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// This function outputs a time profile of joint angles, taking the trajectory
    /// requirements as well as environment settings as input.
    /// Output: 'the_tot' is a time profile of joint angles, 'f_tot' is a time profile of
    /// ducted fan's thrust 
    /// Input: traj_setting(struct), motion_req(struct), envionment_setting
    /// </remarks>
    /// <param name="swing_radius">Input argument #1</param>
    /// <param name="swing_height">Input argument #2</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray crossing_obstacles_with_swing(MWArray swing_radius, MWArray 
                                           swing_height)
    {
      return mcr.EvaluateFunction("crossing_obstacles_with_swing", swing_radius, swing_height);
    }


    /// <summary>
    /// Provides a single output, 3-input MWArrayinterface to the
    /// crossing_obstacles_with_swing MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// This function outputs a time profile of joint angles, taking the trajectory
    /// requirements as well as environment settings as input.
    /// Output: 'the_tot' is a time profile of joint angles, 'f_tot' is a time profile of
    /// ducted fan's thrust 
    /// Input: traj_setting(struct), motion_req(struct), envionment_setting
    /// </remarks>
    /// <param name="swing_radius">Input argument #1</param>
    /// <param name="swing_height">Input argument #2</param>
    /// <param name="mu_kinematic">Input argument #3</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray crossing_obstacles_with_swing(MWArray swing_radius, MWArray 
                                           swing_height, MWArray mu_kinematic)
    {
      return mcr.EvaluateFunction("crossing_obstacles_with_swing", swing_radius, swing_height, mu_kinematic);
    }


    /// <summary>
    /// Provides a single output, 4-input MWArrayinterface to the
    /// crossing_obstacles_with_swing MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// This function outputs a time profile of joint angles, taking the trajectory
    /// requirements as well as environment settings as input.
    /// Output: 'the_tot' is a time profile of joint angles, 'f_tot' is a time profile of
    /// ducted fan's thrust 
    /// Input: traj_setting(struct), motion_req(struct), envionment_setting
    /// </remarks>
    /// <param name="swing_radius">Input argument #1</param>
    /// <param name="swing_height">Input argument #2</param>
    /// <param name="mu_kinematic">Input argument #3</param>
    /// <param name="time_spent_swing">Input argument #4</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray crossing_obstacles_with_swing(MWArray swing_radius, MWArray 
                                           swing_height, MWArray mu_kinematic, MWArray 
                                           time_spent_swing)
    {
      return mcr.EvaluateFunction("crossing_obstacles_with_swing", swing_radius, swing_height, mu_kinematic, time_spent_swing);
    }


    /// <summary>
    /// Provides the standard 0-input MWArray interface to the
    /// crossing_obstacles_with_swing MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// This function outputs a time profile of joint angles, taking the trajectory
    /// requirements as well as environment settings as input.
    /// Output: 'the_tot' is a time profile of joint angles, 'f_tot' is a time profile of
    /// ducted fan's thrust 
    /// Input: traj_setting(struct), motion_req(struct), envionment_setting
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] crossing_obstacles_with_swing(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "crossing_obstacles_with_swing", new MWArray[]{});
    }


    /// <summary>
    /// Provides the standard 1-input MWArray interface to the
    /// crossing_obstacles_with_swing MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// This function outputs a time profile of joint angles, taking the trajectory
    /// requirements as well as environment settings as input.
    /// Output: 'the_tot' is a time profile of joint angles, 'f_tot' is a time profile of
    /// ducted fan's thrust 
    /// Input: traj_setting(struct), motion_req(struct), envionment_setting
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="swing_radius">Input argument #1</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] crossing_obstacles_with_swing(int numArgsOut, MWArray swing_radius)
    {
      return mcr.EvaluateFunction(numArgsOut, "crossing_obstacles_with_swing", swing_radius);
    }


    /// <summary>
    /// Provides the standard 2-input MWArray interface to the
    /// crossing_obstacles_with_swing MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// This function outputs a time profile of joint angles, taking the trajectory
    /// requirements as well as environment settings as input.
    /// Output: 'the_tot' is a time profile of joint angles, 'f_tot' is a time profile of
    /// ducted fan's thrust 
    /// Input: traj_setting(struct), motion_req(struct), envionment_setting
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="swing_radius">Input argument #1</param>
    /// <param name="swing_height">Input argument #2</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] crossing_obstacles_with_swing(int numArgsOut, MWArray swing_radius, 
                                             MWArray swing_height)
    {
      return mcr.EvaluateFunction(numArgsOut, "crossing_obstacles_with_swing", swing_radius, swing_height);
    }


    /// <summary>
    /// Provides the standard 3-input MWArray interface to the
    /// crossing_obstacles_with_swing MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// This function outputs a time profile of joint angles, taking the trajectory
    /// requirements as well as environment settings as input.
    /// Output: 'the_tot' is a time profile of joint angles, 'f_tot' is a time profile of
    /// ducted fan's thrust 
    /// Input: traj_setting(struct), motion_req(struct), envionment_setting
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="swing_radius">Input argument #1</param>
    /// <param name="swing_height">Input argument #2</param>
    /// <param name="mu_kinematic">Input argument #3</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] crossing_obstacles_with_swing(int numArgsOut, MWArray swing_radius, 
                                             MWArray swing_height, MWArray mu_kinematic)
    {
      return mcr.EvaluateFunction(numArgsOut, "crossing_obstacles_with_swing", swing_radius, swing_height, mu_kinematic);
    }


    /// <summary>
    /// Provides the standard 4-input MWArray interface to the
    /// crossing_obstacles_with_swing MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// This function outputs a time profile of joint angles, taking the trajectory
    /// requirements as well as environment settings as input.
    /// Output: 'the_tot' is a time profile of joint angles, 'f_tot' is a time profile of
    /// ducted fan's thrust 
    /// Input: traj_setting(struct), motion_req(struct), envionment_setting
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="swing_radius">Input argument #1</param>
    /// <param name="swing_height">Input argument #2</param>
    /// <param name="mu_kinematic">Input argument #3</param>
    /// <param name="time_spent_swing">Input argument #4</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] crossing_obstacles_with_swing(int numArgsOut, MWArray swing_radius, 
                                             MWArray swing_height, MWArray mu_kinematic, 
                                             MWArray time_spent_swing)
    {
      return mcr.EvaluateFunction(numArgsOut, "crossing_obstacles_with_swing", swing_radius, swing_height, mu_kinematic, time_spent_swing);
    }


    /// <summary>
    /// Provides an interface for the crossing_obstacles_with_swing function in which the
    /// input and output
    /// arguments are specified as an array of MWArrays.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// This function outputs a time profile of joint angles, taking the trajectory
    /// requirements as well as environment settings as input.
    /// Output: 'the_tot' is a time profile of joint angles, 'f_tot' is a time profile of
    /// ducted fan's thrust 
    /// Input: traj_setting(struct), motion_req(struct), envionment_setting
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of MWArray output arguments</param>
    /// <param name= "argsIn">Array of MWArray input arguments</param>
    ///
    public void crossing_obstacles_with_swing(int numArgsOut, ref MWArray[] argsOut, 
                                    MWArray[] argsIn)
    {
      mcr.EvaluateFunction("crossing_obstacles_with_swing", numArgsOut, ref argsOut, 
                                    argsIn);
    }



    /// <summary>
    /// This method will cause a MATLAB figure window to behave as a modal dialog box.
    /// The method will not return until all the figure windows associated with this
    /// component have been closed.
    /// </summary>
    /// <remarks>
    /// An application should only call this method when required to keep the
    /// MATLAB figure window from disappearing.  Other techniques, such as calling
    /// Console.ReadLine() from the application should be considered where
    /// possible.</remarks>
    ///
    public void WaitForFiguresToDie()
    {
      mcr.WaitForFiguresToDie();
    }



    #endregion Methods

    #region Class Members

    private static MWMCR mcr= null;

    private static Exception ex_= null;

    private bool disposed= false;

    #endregion Class Members
  }
}
