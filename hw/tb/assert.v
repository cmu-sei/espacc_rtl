task assert(input test);
   if (test !== 1) begin
      $display("ASSERTION FAILED in %m");
      $finish;
   end
endtask
