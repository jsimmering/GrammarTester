using System;
using System.IO;
using System.Collections.Generic;
using RoboCup.AtHome.CommandGenerator;

namespace RoboCup.AtHome.SPRTest
{
  /// <summary>
  /// Contains the program control logic
  /// </summary>
  public class Program : BaseProgram
  {
    /// <summary>
    /// Random Task generator
    /// </summary>
    protected QuestionsGenerator gen;

    protected override Generator Gen
    {
      get { return this.gen; }
    }

    public Program()
    {
      gen = new QuestionsGenerator();
    }

    /// <summary>
    /// Checks if at least one of the required files are present. If not, initializes the 
    /// directory with example files
    /// </summary>
    public static void InitializePath()
    {
      int xmlFilesCnt = System.IO.Directory.GetFiles (Loader.ExePath, "*.xml", System.IO.SearchOption.TopDirectoryOnly).Length;
      if ((xmlFilesCnt < 4) || !System.IO.Directory.Exists (Loader.GetPath("spr_grammars")))
        ExampleFilesGenerator.GenerateExampleFiles ();
    }

    /// <summary>
    /// Request the user to choose an option for random task generation.
    /// </summary>
    /// <returns>The user's option.</returns>
    protected override char GetOption()
    {
      int opcMin = 1;
      int opcMax = 4;
      ConsoleKeyInfo k;
      Console.WriteLine("Press Esc to quit, c to clear, Enter to generate batch.");
      Console.WriteLine("Choose question category: 1 [p]redefined, 2 [a]rena, 3 c[r]owd, 4 [o]bject");
      do
      {
        k = Console.ReadKey(true);
      } while ((k.Key != ConsoleKey.Escape) && 
          (k.Key != ConsoleKey.Enter) && 
          (k.KeyChar != 'c') && 
          (k.KeyChar != 'p') && 
          (k.KeyChar != 'a') && 
          (k.KeyChar != 'r') && 
          (k.KeyChar != 'o') && 
          ((k.KeyChar < ('0' + opcMin)) || (k.KeyChar > ('0' + opcMax) )));
      if (k.Key == ConsoleKey.Escape)
        return '\0';
      Console.WriteLine(k.KeyChar);
      return k.KeyChar;
    }

    private void PrintQuestions()
    {
      List<Task> questions = gen.GetQuestions();

      // switch Console color to white, backuping the previous one
      ConsoleColor pc = Console.ForegroundColor;
      Console.ForegroundColor = ConsoleColor.White;
      Console.WriteLine();
      // Prints a === line
      string pad = String.Empty.PadRight(Console.BufferWidth - 1, '=');
      Console.WriteLine(pad);
      Console.WriteLine();

      int i = 1;
      foreach (Task task in questions)
      {
        if (task == null) continue;
        string sTask = task.ToString().Trim();
        if (sTask.Length < 1) continue;

        if (String.Equals(sTask, "question", StringComparison.OrdinalIgnoreCase) && (task.Tokens.Count > 0))
          sTask = ExpandPredefinedQuestion(task);
        sTask = sTask.Substring(0, 1).ToUpper() + sTask.Substring(1);
        Console.WriteLine("\t{0}. {1}", i.ToString().PadLeft(2, '0'), sTask);
        ++i;
      }

      Console.WriteLine();
      // Prints another line
      Console.WriteLine(pad);
      // Restores Console color
      Console.ForegroundColor = pc;
      Console.WriteLine();
    }

    private string ExpandPredefinedQuestion(Task task)
    {	
      // 1. Find the question wildcard token
      Token tq = null;
      for(int i = 0; (i < task.Tokens.Count) && (tq == null); ++i)
      {
        // if(task.Tokens[i].IsWildcard && String.Equals(task.Tokens[i].Key, "question", StringComparison.OrdinalIgnoreCase) && (task.Tokens[i].Metadata.Count > 1))
        if(task.Tokens[i].IsWildcard && (task.Tokens[i].Metadata.Count > 1))
          tq = task.Tokens[i];
      }

      // Change how it will be printed
      if(tq == null) return task.ToString().Trim();
      return tq.Metadata[0].Trim();
    }

    /// <summary>
    /// Executes the user's option
    /// </summary>
    /// <param name="opc">User option (category).</param>
    protected override void RunOption(char opc, ref Task task)
    {
      Task t = null;
      switch (opc)
      {
        case '\r': 
        case '\n': 
          PrintQuestions();
          return;
        case '1': case 'p':
          t = gen.GetPredefinedQuestion();
          break;
        case '2': case 'a':
          t = gen.GetArenaQuestion();
          break;
        case '3': case 'r':
          t = gen.GetCrowdQuestion();
          break;
        case '4': case 'o':
          t = gen.GetObjectQuestion();
          break;
        case 'c':
          Console.Clear();
          return;
      }
      if(t != null)
        PrintTask(t);
    }

    /// <summary>
    /// Initializes the random task Generator and loads data from lists and storage
    /// </summary>
    protected override void Setup()
    {
      this.gen = new QuestionsGenerator ();

      Console.ForegroundColor = ConsoleColor.Gray;
      Console.WriteLine ("Question Generator for the Speech & Person Recognition Test");
      Console.WriteLine ();
      base.LoadData();
      Console.WriteLine ();
      Console.WriteLine ();
    }

    /// <summary>
    /// The entry point of the program, where the program control starts and ends.
    /// </summary>
    /// <param name="args">The command-line arguments.</param>
    public static void Main (string[] args)
    {
      InitializePath ();
      if (args.Length == 0)
      {
        new Program ().Run ();
        return;
      }
      ParseArgs(args);
    }

    private static void ParseArgs(string[] args)
    {
      Program p = new Program();
      p.Setup();
      for (int i = 0; i < args.Length; ++i)
      {
        if (args[i] == "--bulk")
          DoBulk(p, args, ref i);
      }
    }


    private static void DoBulk(Program p, string[] args, ref int i)
    {
      int dCount;
      if ((args.Length < (i + 2)) || !Int32.TryParse(args[++i], out dCount) || (dCount < 1)) 
        dCount = 100;

      List<Task> questions = p.gen.GetQuestions(dCount);
      int numQuest = questions.Count;
      Console.WriteLine ("dCount: " + dCount);
      Console.WriteLine ("numQuest: " + numQuest);

      string oDir = String.Format("SPRTest {0} Examples", numQuest);
      if(!Directory.Exists(oDir))
        Directory.CreateDirectory(oDir);
      string oFile = Path.Combine(oDir, String.Format("{0}.txt", oDir));
      using (StreamWriter writer = new StreamWriter(oFile, false, System.Text.Encoding.UTF8)){
        foreach(Task task in questions)
        {
          if (task == null) {
            Console.WriteLine ("task null");
            continue;
          }
          string sTask = task.ToString().Trim();
          if (sTask.Length < 1) {
            Console.WriteLine ("task length < 1");
            continue;
          }
          if (String.Equals(sTask, "question", StringComparison.OrdinalIgnoreCase) && (task.Tokens.Count > 0))
            sTask = p.ExpandPredefinedQuestion(task);
          sTask = sTask.ToLower();
          sTask = sTask.Replace("-", " ");
          sTask = sTask.Replace(".", "");
          sTask = sTask.Replace(",", "");
          sTask = sTask.Replace(";", "");
          sTask = sTask.Replace("!", "");
          sTask = sTask.Replace("?", "");
          sTask = sTask.Replace("q: ", "");

          WriteTaskToFile(writer, task, sTask);
        }
      }

      }

    private static void WriteTaskToFile(StreamWriter writer, Task task, string sTask)
    {
      writer.WriteLine(sTask);
      List<string> remarks = new List<string>();
    }


  }
}
